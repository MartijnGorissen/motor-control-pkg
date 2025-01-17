import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
from my_robot_interfaces.msg import BesturingsData
import can
import struct
from geometry_msgs.msg import Twist
import math


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)         

        # Subscribers voor de BesturingsData message
        self.BesturingsData_subscription = self.create_subscription(BesturingsData, 'besturings_data', self.motor_control_callback, 10)

        # subscriber voor navigatie stuurcommando
        self.twist_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # subscriber voor navigatie mode
        self.nav_mode_subscription = self.create_subscription(Bool, '/nav_mode', self.nav_mode_callback, 10)

        self.throttle = 0
        self.steering = 0
        self.direction = 0
        self.brake = 0

        self.nav_mode = False

        self.timer = self.create_timer(0.04, self.send_can_messages)

    def motor_control_callback(self, msg: BesturingsData):
        self.throttle = int(msg.throttle * 100)  # Scale throttle
        self.brake = int(msg.brake * 100)  # Scale brake
        self.steering = msg.steering
        self.direction = msg.direction
        
    def nav_mode_callback(self, msg: Bool):
        self.nav_mode = msg.data

    def cmd_vel_callback(self, msg: Twist):
        if not self.nav_mode:
            return

        # functie voor het omzetten van /cmd_vel naar data die geschikt is voor de motor

        # Maximale snelheid en stuurhoek
        max_speed = 1

        # Controleer de richting en pas throttle aan
        if msg.linear.x < 0:
            self.direction = 2  # Achteruit
            linear_speed = abs(msg.linear.x)  # Maak snelheid positief
        else:
            self.direction = 1  # Vooruit
            linear_speed = msg.linear.x

        # Zet lineaire snelheid om naar throttle (0-100) PAS OP met het instellen van de versnelling
        # kart begint met rijden bij ongeveer 75-80% throttle
        self.throttle = int(max(0.0, min(100.0, (linear_speed / max_speed) * 100.0)))

        # angular.z is in rad/s, dit moet opgezet worden naar een stuurhoek (ackermann)

        # θ (theta) = gewenste stuurhoek van de voorwielen (in radians)
        # ω = angular.z (in rad/s)
        # v = linear.x (in m/s)
        # wielbasis = 0,31 m
        # θ = arctan (ω * wielbasis / v)

        # maximale stuurhoek wielen 45∘ (-1.25(links) : 1.25(rechts))
        # 1.25 / 45 ≈ 0.0278

        # voorbeeld:
        # ω = 0.5 rad/s 
        # v = 1.0 m/s
        # θ = arctan (0.5 * 0.31 / 1) = arctan (0.155 ) ≈ 0.1549 rad ≈ 8.9∘
        # dus θ = 8.9∘
        # dat schalen naar met het CAN waardes
        # CAN waarde = 8.9∘ * 0.0278 ≈ 0.2475

        # definieer constante
        WHEELBASE = 0.31 # afstand tussen de wielen
        MAX_ANGLE_DEG = 45 
        MAX_CAN_ANGLE = 1.25
        CAN_PER_DEG = MAX_CAN_ANGLE / MAX_ANGLE_DEG

        # als de linear.x (snelheid) is heel klein is kan dit problemen veroorzakan (bijvoorbeeld delen door 0)
        if abs(msg.linear.x) < 1e-5:
            # auto staat (bijna) stil, stuurhoek direct afhankelijk van angular.x
            # verschillende opties, default stuurhoek of geen stuurhoek

            # bij stil staan maximale stuurhoek (verander naar keuze/preferentie)
            # theta = math.copysign(math.radians(MAX_ANGLE_DEG), msg.angular.z)

            # niet sturen bij stil staan
            theta = 0
        else:
            # berekenen stuurhoek achkermann steering
            theta = math.atan((msg.angular.z * WHEELBASE) / msg.linear.x)

        # beperk stuurhoek tot de maximale hoek
        max_angle_rad = math.radians(MAX_ANGLE_DEG)
        if theta > max_angle_rad:
            theta = max_angle_rad
        elif theta < -max_angle_rad:
            theta = -max_angle_rad

        # rad -> gaden
        angle_deg = math.degrees(theta)

        # graden -> stuurwaarde (CAN) (45graden = 1.25)
        # inverteer de waarde want angular.z(positief) is naar links. 
        # om de auto naar links te laten sturen is dit negatief bij ons. dus -angle_deg
        self.steering = -angle_deg * CAN_PER_DEG

        print(f"Throttle: {self.throttle}, Steering: {self.steering}, Direction {self.direction}")


    def send_can_messages(self):
        # uncomment brake message bij echt karte
        
        # brk_msg = can.Message(arbitration_id=0x110, data=[self.brake, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
        steering_msg = can.Message(arbitration_id=0x220, data=list(bytearray(struct.pack("f", self.steering))) + [0, 0, 0, 0], is_extended_id=False)
        acc_msg = can.Message(arbitration_id=0x330, data=[self.throttle, 0, self.direction, 0, 0, 0, 0, 0], is_extended_id=False)
        
        # self.bus.send(brk_msg)
        self.bus.send(steering_msg)
        self.bus.send(acc_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

cmake_minimum_required(VERSION 3.5)
project(motor_control_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(
  PROGRAMS
    motor_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install message files (optional but good practice)
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

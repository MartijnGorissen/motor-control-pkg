from setuptools import setup

package_name = 'motor_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sdc',
    maintainer_email='sdc@todo.todo',
    description='Package for motor control using ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control_node = motor_control_pkg.motor_control_node:main',
        ],
    },
)

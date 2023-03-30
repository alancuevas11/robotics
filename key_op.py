#!/usr/bin/env python3
"""
This module defines a ROS node that publishes Twist messages based
on keyboard inputs.

It uses the KeysToVelocities class from the robot_model module to map
keyboard inputs to linear and angular velocities.
The me416_utilities module is used to get keyboard inputs.

Keyboard commands:

'w': Increase linear speed
's': Decrease linear speed
'a': Increase angular speed
'd': Decrease angular speed
'z': Stop linear motion
'c': Stop angular motion
'x': Stop all motion
'q': Shutdown Node
The node publishes Twist messages on the robot_twist topic 
with the current linear and angular velocities.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_model import KeysToVelocities
import me416_utilities as mu

class TwistPublisher(Node):
    """
    A ROS node that publishes Twist messages based on keyboard inputs.
    """
    def __init__(self):
        """
        Initializes the TwistPublisher node.
        """
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(Twist, 'robot_twist', 10)
        self.keys_to_velocities = KeysToVelocities()
        print("""Keyboard commands:
        - 'w': Increase linear speed
        - 's': Decrease linear speed
        - 'a': Increase angular speed
        - 'd': Decrease angular speed
        - 'z': Stop linear motion
        - 'c': Stop angular motion
        - 'x': Stop all motion
        - 'q or Q': Shutdown Node""")
        self.getch = mu._Getch()
        self.is_running = True

    def run(self):
        """
        Publishes Twist messages based on keyboard inputs.
        """
        key = self.getch()
        if key == 'q' or key=='Q':
            self.get_logger().info('Shutdown initiated by user')
            self.destroy_node()
            self.is_running = False
        else:
            key = self.getch()
            speed_linear, speed_angular, text_description = \
            self.keys_to_velocities.update_speeds(key)
            print(text_description)
            print(f"Linear speed Value: {speed_linear}, Angular speed value: {speed_angular}")
            twist_msg = Twist()
            twist_msg.linear.x = speed_linear
            twist_msg.angular.z = speed_angular
            self.publisher_.publish(twist_msg)

def main(args=None):
    """
    The main function that initializes and runs the TwistPublisher node.
    """
    rclpy.init(args=args)
    node = TwistPublisher()
    while node.is_running:
        node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

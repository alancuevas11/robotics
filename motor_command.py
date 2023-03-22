#!/usr/bin/env python3
"""
This script contains a ROS2 node that subscribes to the 'robot_twist' topic
to receive a Twist message and uses the'robot_model' module to convert the
Twist message to motor speeds for the left and right motors of a robot. The
left and right motor speeds are then published to the 'motor_speeds' topic
using the 'MotorSpeedsStamped' message type. This script also uses the
'me416_utilities' module to create objects for the left and right motors and
set their speeds.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from  me416_msgs.msg import MotorSpeedsStamped
import me416_utilities
import robot_model

#Create objects for left and right motor
MOTOR_LEFT = me416_utilities.MotorSpeedLeft()
MOTOR_RIGHT = me416_utilities.MotorSpeedRight()

class MotorCommand(Node):
    '''
    Simple Node class that uses a timer to publish on /chatter
    '''
    def __init__(self):
        '''
        Setup subscriber and publisher
        '''
        super().__init__('motor_command')
        self.subscriber = self.create_subscription(Twist, 'robot_twist',
                                                   self.twist_callback, 10)
        self.publisher = self.create_publisher(MotorSpeedsStamped, 'motor_speeds', 10)

    def twist_callback(self, msg: Twist):
        """
        Callback to receive a Twist and set motor speeds for left and right motor
        """
        #Take content from Twist msg and turn it into speeds for left and righ
        speed_linear = msg.linear.x
        speed_angular = msg.angular.z
        left_speed, right_speed = robot_model.twist_to_speeds(speed_linear, speed_angular)
        left_speed_double = float(left_speed)
        right_speed_double = float(right_speed)
        #set motor speeds
        MOTOR_LEFT.set_speed(left_speed_double)
        MOTOR_RIGHT.set_speed(right_speed_double)
        #create message
        motor_speeds_msg = MotorSpeedsStamped()
        motor_speeds_msg.header.stamp = self.get_clock().now().to_msg()
        motor_speeds_msg.left = left_speed
        motor_speeds_msg.right = right_speed
        # Publish the message
        self.publisher.publish(motor_speeds_msg)
def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    minimal_publisher = MotorCommand()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly, otherwise
    # it will be done automatically
    minimal_publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

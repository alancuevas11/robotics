#!/usr/bin/env/python3
"""
Publishes odometry
"""

from math import pi

import me416_utilities as mu
import numpy as np
import rclpy
import robot_model as rm
from geometry_msgs.msg import PoseStamped
from me416_msgs.msg import MotorSpeedsStamped
from rclpy.node import Node
from transforms3d.euler import euler2quat


class Odometry(Node):
    '''
    Publishes a pose using Euler integration from motor speeds
    '''
    def __init__(self):
        super().__init__('odometry_encoders')
        self.publisher = self.create_publisher(MotorSpeedsStamped, 'odom_euler',10)
        self.subscriber= self.create_subscription(PoseStamped, 'encoders', self.encoders_callback, 10)       
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.z_state= 0
        self.register = rm.StampedMsgRegister()

    def timer_callback(self):
        '''
        Publish current pose estimate
        '''
        msg = PoseStamped()
        # Fill orientation
        theta = self.z_state
        quat = euler2quat(0, 0, theta)
        msg.pose.orientation.x = quat[1]
        msg.pose.orientation.y = quat[2]
        msg.pose.orientation.z = quat[3]
        msg.pose.orientation.w = quat[0]
        # Fill position
        # TODO: set msg.pose.position using the current state
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        self.publisher.publish(msg)

    def encoders_callback(self, msg):
        '''
        Integrate the motor speeds in pose estimate using Euler's method
        '''
        # TODO: obtain elapsed time from previous call using the StampedMsgRegister attribute
        # Note: the elapsed time cannot will not be available on the first call
        # TODO: obtain input u for the model from msg
        # TODO: apply Euler step to update current state estimate


def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

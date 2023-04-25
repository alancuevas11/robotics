#!/usr/bin/env/python3
"""
Publishes odometry using Euler integration from motor speeds.
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
        self.publisher = self.create_publisher(PoseStamped, 'odom_euler',10)
        self.subscriber= self.create_subscription(MotorSpeedsStamped, 'encoders',
                                                  self.encoders_callback, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.z_state= 0  # Initialize state estimate to zero vector
        self.register = rm.StampedMsgRegister()  # Create object to keep track of time

    def timer_callback(self):
        '''
        Publish current pose estimate
        '''
        msg = PoseStamped()
        # Fill orientation
        theta = self.z_state  # Extract orientation from state estimate
        quat = euler2quat(0, 0, theta)  # Convert Euler angles to quaternion
        msg.pose.orientation.x = quat[1]  # Fill quaternion components
        msg.pose.orientation.y = quat[2]
        msg.pose.orientation.z = quat[3]
        msg.pose.orientation.w = quat[0] 
        msg.pose.position.x = self.z_state[0]  # Fill position components
        msg.pose.position.y = self.z_state[1]
        msg.pose.position.z = 0  # Set z position to zero (2D planar case, no change in height)
        msg.header.stamp = self.get_clock().now().to_msg()  # Get current time and set message header
        msg.header.frame_id = 'map'  # Set frame ID
        self.publisher.publish(msg)  # Publish message
        
    def encoders_callback(self, msg):
        '''
        Integrate the motor speeds in pose estimate using Euler's method
        '''
        stepsize = rm.StampedMsgRegister.replace_and_compute_delay(self,msg)  # Compute time step between messages
        speed_linear = msg.twist.linear.x  # Extract linear speed from message
        speed_angular = msg.twist.angular.z  # Extract angular speed from message
        # Convert linear and angular speeds to motor speeds
        u_lw, u_rw = rm.twist_to_speeds(speed_linear, speed_angular)
        u_current = np.array([u_lw, u_rw])
        z_next = rm.euler_step(self.z_state, u_current, stepsize)  # Integrate state estimate using Euler's method
        self.z_state= z_next  # Update state estimate

def main(args=None):
    '''
    Initialize ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

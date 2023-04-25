#!/usr/bin/env/python3
"""
Publishes odometry using Euler integration from motor speeds.
"""
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
        # Initialize state estimate to zero vector
        self.z_state= 0
        # Create object to keep track of time
        self.register = rm.StampedMsgRegister()

    def timer_callback(self):
        '''
        Publish current pose estimate
        '''
        msg = PoseStamped()
        # Fill orientation
        # Extract orientation from state estimate
        theta = self.z_state
        # Convert Euler angles to quaternion
        quat = euler2quat(0, 0, theta)
        # Fill quaternion components
        msg.pose.orientation.x = quat[1]
        msg.pose.orientation.y = quat[2]
        msg.pose.orientation.z = quat[3]
        msg.pose.orientation.w = quat[0]
        # Fill position components
        msg.pose.position.x = self.z_state[0]
        msg.pose.position.y = self.z_state[1]
        # Set z position to zero (2D planar case, no change in height)
        msg.pose.position.z = 0
        # Get current time and set message header
        msg.header.stamp = self.get_clock().now().to_msg()
         # Set frame ID
        msg.header.frame_id = 'map'
        # Publish message
        self.publisher.publish(msg)
    def encoders_callback(self, msg):
        '''
        Integrate the motor speeds in pose estimate using Euler's method
        '''
        # Compute time step between messages
        stepsize = rm.StampedMsgRegister.replace_and_compute_delay(self,msg)
        # Extract linear speed from message
        speed_linear = msg.twist.linear.x
        # Extract angular speed from message
        speed_angular = msg.twist.angular.z
        # Convert linear and angular speeds to motor speeds
        u_lw, u_rw = rm.twist_to_speeds(speed_linear, speed_angular)
        u_current = np.array([u_lw, u_rw])
        # Integrate state estimate using Euler's method
        z_next = rm.euler_step(self.z_state, u_current, stepsize)
        # Update state estimate
        self.z_state= z_next

def main(args=None):
    '''
    Initialize ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

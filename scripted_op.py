#!/usr/bin/env python3
"""
This script creates a ROS2 node that publishes a sequence of linear and
angular speed commands for the ROSbot to follow. The speed commands are defined
as a list of pairs of linear and angular speeds in the COMMANDS constant.

The node publishes Twist messages on the robot_twist topic at a fixed interval
of 1Hz. When the end of the COMMANDS list is reached, the node returns
to the beginning of the list and starts again.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# A list of pairs of linear and angular speeds
COMMANDS = [
    (1.0, 0.0),   # forward
    (1.0, 0.0),   # forward
    (0.0, 0.0),   # stop
    (0.0, 1.57079635),   # turn
    (0.0, 0.0),   # stop
]

class ScriptedOp(Node):
    """
    A ROS node that publishes Twist messages
    """
    def __init__(self):
        super().__init__('scripted_op_publisher')
        self.publisher = self.create_publisher(Twist, 'robot_twist',10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.cmd_index = 0
    def timer_callback(self):
        """
        Timer callback function that gets called every 1 second to publish
        the next linear and angular speed command from the COMMANDS list.
        """
        # Get the current command from the list
        speed_linear, speed_angular = COMMANDS[self.cmd_index]
        # Publish Twist message with current command
        twist_msg = Twist()
        twist_msg.linear.x = speed_linear
        twist_msg.angular.z = speed_angular
        self.publisher.publish(twist_msg)
        # Increment the command index and wrap around to the
        # beginning of the list if we've reached the end
        self.cmd_index = (self.cmd_index + 1) % len(COMMANDS)
def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    minimal_publisher = ScriptedOp()
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly, otherwise
    # it will be done automatically
    minimal_publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    
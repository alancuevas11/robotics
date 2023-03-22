#!/usr/bin/env python
"""
Simple repeater demo that receives std_msgs/Strings messages from the 'chatter'
 topic, modifies them, and then sends them on the 'chatter_repeated' topic
"""
import time
import rclpy
from std_msgs.msg import String
message_buffer = []
start_time = None

def callback(msg):
    """ Callback to receive a message and concatenate it with 
    other messages received within a span of 3 seconds """
    global pub, repeater, message_buffer, start_time

    # Check if this is the first message
    if not start_time:
        start_time = time.time()

    # Append the message to the buffer
    message_buffer.append(msg.data)

    # Check if 3 seconds have elapsed since the first message
    if time.time() - start_time >= 3.0:

        # Concatenate the messages in the buffer
        concatenated_message = ', '.join(message_buffer)

        # Publish a new message with the concatenated string
        msg_repeated = String()
        msg_repeated.data = concatenated_message
        pub.publish(msg_repeated)

        # Reset the buffer and start time for the next set of messages
        message_buffer = []
        START_TIME = None

        # Show some logging information (optional)
        repeater.get_logger().info('Concatenated messages: ' + concatenated_message)

def main(args=None):
    """Node setup and main ROS loop"""
    # Init node. anonymous=True allows multiple launch with automatically assigned names
    rclpy.init(args=args)
    repeater = rclpy.create_node('repeater')
    # Prepare subscriber on a topic, and publisher on another
    repeater.create_subscription(String, 'chatter', callback, 10)
    pub = repeater.create_publisher(String, 'chatter_repeated', 10)
    rclpy.spin(repeater)
    # node cleanup
    repeater.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass

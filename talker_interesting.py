#!/usr/bin/env python3
"""
Simple demo that publishes to std_msgs/Strings published
to the 'chatter' topic
"""


import rclpy
from std_msgs.msg import String


TEAMS = ['Illinois Fighting Illini', 'Baylor Bears', 'Michigan Wolverines',
         'Gonzaga Bulldogs', 'Alabama Crimson Tide', 'Houston Cougars',
         'Ohio State Buckeyes', 'Iowa Hawkeyes', 'Texas Longhorns',
         'West Virginia Mountaineers', 'Kansas Jayhawks',
         'Arkansas Razorbacks', 'Florida State Seminoles', 'Purdue Boilermakers',
         'Texas Tech Red Raiders', 'Oklahoma State Cowboys', 'Loyola Chicago Ramblers',
         'Oregon State Beavers', 'Creighton Bluejays',
         'Virginia Cavaliers', 'Colorado Buffaloes', 'San Diego State Aztecs',
         'USC Trojans', 'Virginia Tech Hokies', 'North Carolina Tar Heels', 'Tennessee Volunteers',
         'Clemson Tigers', 'Wisconsin Badgers', 'Georgetown Hoyas',
         'BYU Cougars', 'UCLA Bruins', 'Rutgers Scarlet Knights']

def timer_callback():
    '''
    Function called by the Timer object
    '''
    global node, i, publisher
    msg = String()
    msg.data = f'{TEAMS[i % len(TEAMS)]}'
    i += 1
    node.get_logger().info(f'Publishing: {msg.data}')
    publisher.publish(msg)

def main(args=None):
    '''
    Init ROS, setup timer, spin, cleanup
    '''
    global node, i, publisher
    rclpy.init(args=args)
    i = 0
    node = rclpy.create_node('talker')
    publisher = node.create_publisher(String, 'chatter', 10)
    timer_period = 0.5  # seconds
    timer = node.create_timer(timer_period, timer_callback)
    rclpy.spin(node)
    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

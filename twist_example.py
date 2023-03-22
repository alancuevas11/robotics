""" Example of how to set attributes in ROS messages """
from geometry_msgs.msg import Twist


def twist_fill():
    # This is a stub. Substitute with your own code
    msg = Twist()
    msg.linear.x = 2.0
    msg.linear.y = 3.0
    msg.linear.z = 2.0
    msg.angular.x = 1.0
    msg.angular.y = 2.0
    msg.angular.z = 1.0
    return msg


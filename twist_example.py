""" Example of how to set attributes in ROS messages """
from geometry_msgs.msg import Twist


def twist_fill():
    # This is a stub. Substitute with your own code
    msg = Twist()
    msg.linear.x = float(0.70)
    msg.angular.z = float(0.10)
    return msg


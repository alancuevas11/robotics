"""Functions for modeling ROSBot"""
#from math import cos, sin
#import numpy as np
def model_parameters():
    """Returns two constant model parameters"""
    param_k = 1.0
    param_d = 0.5
    return param_k, param_d
def twist_to_speeds(speed_linear, speed_angular):
    """until we learn the actual kinematics, rights and left motor speed will be
    approximated as the sum/difference between the linear speed and the angular
    speed*the linear speed"""
    right = speed_linear + speed_angular*speed_linear
    left = speed_linear - speed_angular*speed_linear
    #velocities are normalized based on the hightest velocity
    # to ensure speeds are less than or equal to 1
    max_vel = max(abs(left), abs(right))
    if max_vel > 1:
        right = right/max_vel
        left = left/max_vel
    #returns given speed values
    return left, right

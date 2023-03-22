
"""Functions for modeling ROSBot"""

from math import cos, sin

import numpy as np


def model_parameters():
    """Returns two constant model parameters"""
    param_k = 1.0
    param_d = 0.5
    return param_k, param_d


#def system_matrix(theta):
    """Returns a numpy array with the A(theta) matrix for a differential drive robot"""
 #   return a_matrix


#def system_field(z, u):
    """Computes the field at a given state for the dynamical model"""
#    return dot_z


#def euler_step(z, u, stepSize):
   # """Integrates the dynamical model for one time step using Euler's method"""
   # return z_next


def twist_to_speeds(speed_linear, speed_angular):
    right = speed_linear + speed_angular
    left = speed_linear - speed_angular
    max_vel = max(abs(left),abs(right))
    if max_vel > 1:
        right = right/max_vel
        left = left/max_vel 
    return left, right

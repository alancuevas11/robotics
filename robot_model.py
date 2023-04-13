"""Functions for modeling ROSBot"""

import math
import numpy as np
import me416_utilities as mu

def model_parameters():
    """Returns two constant model parameters"""
    k = 1.0
    d = 0.5
    return k, d

def twist_to_speeds(speed_linear, speed_angular):
    '''Modified twist_to_speeds version'''
    k, d = model_parameters()
    # Calculate the angular velocity
    theta = math.atan(speed_linear)
    # Calculate the A(theta) matrix
    A = system_matrix(theta)
    # Create z_dot
    z_dot = np.array([[speed_linear*math.cos(theta)],
                      [speed_linear*math.sin(theta)],
                      [speed_angular]])
    # Solve for u matrix
#    u = np.linalg.solve(A, z_dot)
    # Extract the left and right motor speeds
    u_lw= (1/k) * (speed_linear+ (d * speed_angular))
    u_rw = (1/k) * (((-1*d)*speed_angular) +speed_linear)
    u_lw = max(u_lw, -1.0)
    u_lw = min(u_lw, 1.0)
    u_rw = max(u_rw, -1.0)
    u_rw = min(u_rw, 1.0)
    # Return the left and right motor speeds
    return u_lw, u_rw


def system_matrix(theta):
    """Returns a numpy array with the A(theta) matrix for a differential drive robot"""
    k, d = model_parameters()
    A = np.array([[(k/2)*math.cos(theta), (k/2)*math.cos(theta)],
                  [(k/2)*math.sin(theta), (k/2)*math.sin(theta)],
                  [(k/2)*(-1/d), (k/2)*(1/d)]])
    return A


def euler_step(z_current, u_current, step_size):
    """Integrates the dynamical model for one time step using Euler's method"""
    A = system_matrix(z_current[2])
    z_next = z_current + step_size*A@u_current
    return z_next

class KeysToVelocities:
    '''Setting up the KTV class'''
    def __init__(self):
        self.speed_linear = 0.0
        self.speed_angular = 0.0
        self.SPEED_DELTA = 0.2

    def update_speeds(self, key):
        '''Updating linear and angular speed'''
        text_description = ''

        if key == 'w' or key == 'W':
            self.speed_linear = min(self.speed_linear + self.SPEED_DELTA, 1.0)
            text_description = 'Increase linear speed'
        elif key == 's' or key == 'S':
            self.speed_linear = max(self.speed_linear - self.SPEED_DELTA, -1.0)
            text_description = 'Decrease linear speed'
        elif key == 'a' or key == 'A':
            self.speed_angular = min(self.speed_angular + self.SPEED_DELTA, 1.0)
            text_description = 'Increase angular speed'
        elif key == 'd' or key == 'D':
            self.speed_angular = max(self.speed_angular - self.SPEED_DELTA, -1.0)
            text_description = 'Decrease angular speed'
        elif key == 'z' or key == 'Z':
            self.speed_linear = 0.0
            text_description = 'Linear speed set to zero'
        elif key == 'c' or key =='C':
            self.speed_angular = 0.0
            text_description = 'Angular speed set to zero'
        elif key == 'x' or key =='X':
            self.speed_angular = 0.0
            self.speed_linear = 0.0
            text_description = 'Linear and Angular speeds set to zero'
        else:
            text_description = 'Invalid command'
        return self.speed_linear, self.speed_angular, text_description

class StampedMsgRegister:
    '''Computes the delay between two ROS messages'''
    def __init__(self):
        '''Initializes the internal first message'''
        self.msg_previous = None

    def replace_and_compute_delay(self, msg):
        '''Computes the delay between new input message and previous message'''
        if self.msg_previous is None:
           time_delay = None
        else:
            time_delay = mu.stamp_difference(msg.header.stamp, self.msg_previous.header.stamp)
        msg_previous = self.msg_previous
        self.msg_previous = msg
        return time_delay, msg_previous

    def previous_stamp(self):
        '''Returns the time stamp of msg_previous'''
        if self.msg_previous is None:
            return None
        return self.msg_previous.header.stamp

"""Functions for modeling ROSBot"""
#from math import cos, sin
#import numpy as np
import me416_utilities as mu

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

class KeysToVelocities(object):
    '''
    Class to translate cumulative key strokes to speed commands
    '''
    def __init__(self):
        '''
        Initialize the KeysToVelocities object with the initial values
        of the speed and other attributes.
        
        Attributes:
        - speed_linear (float): the linear speed of the robot, initially set to 0.0
        - speed_angular (float): the angular speed of the robot, initially set to 0.0
        - SPEED_DELTA (float): the increment of the speed when a key is pressed, set to 0.2
        - text_description (string): a text description of the current state of the robot,
        initially set to "Ready"
        - getch (mu._Getch): an instance of the _Getch class from the me416_utilities module
        '''
        # initialize attributes here
        self.speed_linear = 0.0
        self.speed_angular = 0.0
        self.SPEED_DELTA = 0.2
        self.text_description = "Ready"

    def update_speeds(self, key):
        '''
        Update the linear and angular speeds of the robot according to the pressed key.

        Args:
        - key (str): the key pressed by the user

        Returns:
        - speed_linear (float): the updated linear speed of the robot
        - speed_angular (float): the updated angular speed of the robot
        - text_description (string): a text description of the current state of the robot
        '''
        # Map keys to actions
        if not key.islower():
            print("Input key must be lowercase")
        if key == 'w':
            self.speed_linear = float(min(self.speed_linear + self.SPEED_DELTA, 1.0))
            self.text_description = "Increased linear speed"
        elif key == 's':
            self.speed_linear = float(max(self.speed_linear - self.SPEED_DELTA, -1.0))
            self.text_description = "Decreased linear speed"
        elif key == 'a':
            self.speed_angular = float(min(self.speed_angular + self.SPEED_DELTA, 1.0))
            self.text_description = "Increased angular speed"
        elif key == 'd':
            self.speed_angular = float(max(self.speed_angular - self.SPEED_DELTA, -1.0))
            self.text_description = "Decrease angular speed"
        elif key == 'z':
            self.speed_linear = 0.0
            self.text_description = "Stopped linear motion"
        elif key == 'c':
            self.speed_angular = 0.0
            self.text_description = "Stopped angular motion"
        elif key == 'x':
            self.speed_linear = 0.0
            self.speed_angular = 0.0
            self.text_description = "Stopped all motion"
        else:
            self.text_description = "Invalid Command"

        return self.speed_linear, self.speed_angular, self.text_description

#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Joint
"""

import os
import sys
import time
import math
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

#def flatten_list(nested_list):
    #return [item for sublist in nested_list for item in sublist]


def get_robot_coordinates(ip):
    # Initialize the robot
    arm = XArmAPI(ip)
    arm.connect()

    # Ensure the robot is ready
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)

    # Move the robot to some position (example)
    arm.set_servo_angle(angle=[0, -30, -60, 0, 0, 0, 0], speed=20, wait=True)
    status, joint_coords = arm.get_servo_angle()  ##it gives us an tuple, status is the first element and the coordinates is the second element

    if status != 0:
        print(f"Error in retrieving joint coordinates, status: {status}")
        return [0.0, 0.0]

    # Disconnect the robot
    arm.disconnect()

    # Return the coordinates
    #return joint_coords
    #joint_coords_str= ', '.join(map(str,joint_coords))
    ##joint_coords = [float(coord) if not isinstance(coord, list) else float(coord[0]) for coord in joint_coords]
    joint_coords = np.array(joint_coords, dtype=float)
    return joint_coords
    



# Example test
if __name__ == "__main__":
    # Replace '192.168.1.100' with your robot's actual IP address
    print(get_robot_coordinates('192.168.1.100'))

#######################################################


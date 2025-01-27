import numpy as np

# global coordinate in Cartesian
i = np.array([1, 0, 0])
j = np.array([0, 1, 0])
k = np.array([0, 0, 1])

# This section requires measurements and should be perpendicular
starting_point = np.array([306.2, 1.8, 63.4]) #H transform matrix away from base
x = np.array([1, 0, 0])
y = np.array([0, 1, 0])
z = np.cross(x, y)

# Compute the transformed basis
i_prime = (x - starting_point) / np.linalg.norm(x - starting_point)
j_prime = (y - starting_point) / np.linalg.norm(y - starting_point)
if not np.isclose(np.dot(i_prime, j_prime), 0):
    print("not perpendicular")
k_prime = z / np.linalg.norm(z)

# Compute the rotation matrix R
R = np.array([
    [np.dot(i, i_prime), np.dot(j, i_prime), np.dot(k, i_prime)],
    [np.dot(i, j_prime), np.dot(j, j_prime), np.dot(k, j_prime)],
    [np.dot(i, k_prime), np.dot(j, k_prime), np.dot(k, k_prime)]
])

# Original point in Cartesian coordinates
pinO = np.array([0, 0, 0])
# Transform from original to target coordinate system
trans = np.dot(R, (pinO - starting_point))

transB =  np.dot(R.T, trans) + starting_point #from target coordinate system to global, trans can be arbitary point in target coordinate 

print(trans)

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.append("C:\\Users\\23149\\Documents\\GitHub\\xArm-Python-SDK\\xarm\\x3")
from xarm.wrapper import XArmAPI
import math
from xarm.x3 import XArm, Studio

port = '192.197.1.168'

arm = XArmAPI(port)
arm.connect()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
arm.set_position(x=trans[0], y=trans[1], z=trans[2], roll=-180, pitch=0, yaw=0, speed=50,is_radian=False,wait=True)
print(arm.get_position())
arm.disconnect()
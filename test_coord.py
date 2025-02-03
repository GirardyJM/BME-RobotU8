import numpy as np
import math

# global coordinate in base
i = np.array([1, 0, 0])
j = np.array([0, 1, 0])
k = np.array([0, 0, 1])

# This section requires measurements and should be perpendicular
starting_point = np.array([148, 1.8, 64.6]) #initial position to base
x = np.array([1, 0, 0]) #this input should be calculated by readout real-world position from digitalizer
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

# Any point in either coordinate
pinO = np.array([0, 0, 40])

# Transform from global to target coordinate
trans = np.dot(R, (pinO - starting_point))

#from target coordinate system to global, pinOcan be arbitary point in target coordinate 
transB =  np.dot(R.T, pinO) + starting_point 

print(transB)

#angles between tibia and fermur
#get two coordinates first and take transformation matrix
#test on getting angles; to calculate need distance between tibia and fermur
x = np.array([0.5**0.5, 0.5**0.5, 0]) #this input should be calculated by readout real-world position from digitalizer
y = np.array([-0.5**0.5, 0.5**0.5, 0])
z = np.cross(x, y)
R = np.array([
    [np.dot(i, x), np.dot(j, x), np.dot(k, x)],
    [np.dot(i, y), np.dot(j, y), np.dot(k, y)],
    [np.dot(i, z), np.dot(j, z), np.dot(k, z)]
])
adduction = math.acos(R[0,2])-0.5*np.pi #default input as rad
flexion = math.atan(R[1,2]/R[2,2])
rotation = math.atan(R[0,1]/R[0,0])*180/np.pi #change from rad to degree
print(adduction)
print(flexion)
print(rotation)


import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.append("C:\\Users\\23149\\Documents\\GitHub\\xArm-Python-SDK\\xarm\\x3")
from xarm.wrapper import XArmAPI
import math
from xarm.x3 import XArm, Studio

port = '192.168.1.197'

arm = XArmAPI(port)
arm.connect()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
#arm.set_position(x=starting_point[0],y=starting_point[1],z=starting_point[2],roll=-180,pitch=0,yaw=0,speed=50,is_radian=False,wait = True)
#arm.set_position(x=transB[0], y=transB[1], z=transB[2], roll=-180, pitch=0, yaw=0, speed=50,is_radian=False,wait=True)
arm.set_position(x=595.8,y=-8.1,z=312.8,roll=-180,pitch=0,yaw=0,speed=50,is_radian=False,wait = True)
arm.set_gripper_enable(1)
arm.set_gripper_position(104)
'''arm.set_gripper_enable(...)
arm.set_gripper_mode(...)
arm.set_gripper_speed(...)
arm.set_gripper_position(...)
arm.get_gripper_position()
arm.get_gripper_err_code()
arm.clean_gripper_error()'''
print(arm.get_position())
arm.disconnect()
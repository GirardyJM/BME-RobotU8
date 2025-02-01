import numpy as np
import sys
import os
import time
import math
from xarm.x3 import XArm, Studio
from xarm.wrapper import XArmAPI

port = '192.168.1.197'

arm = XArmAPI(port)

arm.connect()
# global coordinate in base
i = np.array([1, 0, 0])
j = np.array([0, 1, 0])
k = np.array([0, 0, 1])

# This section requires measurements and should be perpendicular
starting_point = np.array([629, -2.8, 52]) #initial position to base
pose = arm.get_position() # Ex: this  (0, [222.560226, 1.77597, 64.591461, 180.00002, 0.005558, 0.005443])
current_roll, current_pitch, current_yaw = pose[1][3], pose[1][4], pose[1][5]
print(f"Current Roll: {current_roll}, Pitch: {current_pitch}, Yaw: {current_yaw}")
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
    [np.dot(i, x), np.dot(j, x), np.dot(k, x)],
    [np.dot(i, y), np.dot(j, y), np.dot(k, y)],
    [np.dot(i, z), np.dot(j, z), np.dot(k, z)]
])

# Any point in either coordinate
pinO = np.array([0, 0, 0])

# Transform from global to target coordinate
trans = np.dot(R, (pinO - starting_point))

#from target coordinate system to global, pinOcan be arbitary point in target coordinate 
transB =  np.dot(R.T, pinO) + starting_point 


print(transB)
#Need to be modified and fixed does not accurately rotate model with translational and rotational movement 
def rotation_matrix(axis, theta_degrees): #with just the torque sens
    """Returns a 3D rotation matrix for a given axis ('x', 'y', or 'z') and angle in degrees."""
    theta = np.radians(theta_degrees)  # Convert degrees to radians
    
    if axis == 'x':
        R = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])
    elif axis == 'y':
        R = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
    elif axis == 'z':
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'.")
    
    return R

def matrix_to_euler(R):
    """Convert a rotation matrix back to Euler angles (roll, pitch, yaw)."""
    roll_new = math.atan2(R[2, 1], R[2, 2])  # Rotation around X-axis
    pitch_new = math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2))  # Rotation around Y-axis
    yaw_new = math.atan2(R[1, 0], R[0, 0])  # Rotation around Z-axis

    return np.degrees(roll_new), np.degrees(pitch_new), np.degrees(yaw_new)


R_local = rotation_matrix('x', 40)

R_current = rotation_matrix('x', current_roll) @ rotation_matrix('y', current_pitch) @ rotation_matrix('z', current_yaw)

R_new = R_local @ R_current

# Convert rotated matrix to Euler angles
roll_new, pitch_new, yaw_new = matrix_to_euler(R_new)



# Define rotation axis and angle
#theta_degrees = 20  # Change to your desired rotation angle
#rotation_axis = 'y'  # Choose 'x', 'y', or 'z'
# Get the corresponding rotation matrix
#R = rotation_matrix(rotation_axis, theta_degrees) #if you are not using gripper
#roll_new, pitch_new, yaw_new= rotation_matrix_grip(rotation_axis, theta_degrees) 
# Apply rotation to transB
transB = np.dot(R, transB - starting_point) + starting_point  # Rotate relative to base , for when you arent using the gripper
#transB_rotated = np.dot(R, transB - starting_point) + starting_point



arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
arm.set_gripper_position(500)
arm.move_gohome()
arm.set_gripper_enable(1)
arm.set_gripper_position(300)
arm.set_position(x=starting_point[0],y=starting_point[1],z=starting_point[2],roll=-180,pitch=0,yaw=0,speed=50,is_radian=False,wait = True)
arm.set_gripper_position(119)
#arm.set_position(x=transB[0], y=transB[1], z=transB[2], roll=-180, pitch=0, yaw=0, speed=30,is_radian=False,wait=True)
#arm.set_position(x=transB_rotated[0], y=transB_rotated[1], z=transB_rotated[2], roll=-180, pitch=0, yaw=0, speed=30,is_radian=False,wait=True) # without gripper
arm.set_position(x=starting_point[0],y=starting_point[1],z=starting_point[2],roll=roll_new,pitch=pitch_new,yaw=yaw_new,speed=50,is_radian=False,wait = True)
#arm.set_position(x=595.8,y=-8.1,z=312.8,roll=-180,pitch=0,yaw=0,speed=50,is_radian=False,wait = True)

'''arm.set_gripper_enable(...)
arm.set_gripper_mode(...)
arm.set_gripper_speed(...)
arm.set_gripper_position(...)
arm.get_gripper_position()
arm.get_gripper_err_code()
arm.clean_gripper_error()'''
print(f"Gripper rotated: roll={roll_new}, pitch={pitch_new}, yaw={yaw_new}")
print("New Gripper Local Basis:")
print(f"Updated Euler Angles: Roll={roll_new:.2f}, Pitch={pitch_new:.2f}, Yaw={yaw_new:.2f}")
print(arm.get_position())
#print("Robot moved to rotated position:", transB_rotated)
arm.disconnect()

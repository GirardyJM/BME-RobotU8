import numpy as np
import math
from xarm.x3 import XArm, Studio
from xarm.wrapper import XArmAPI

# global coordinate in base
i = np.array([1, 0, 0])
j = np.array([0, 1, 0])
k = np.array([0, 0, 1])

# This section requires measurements and should be perpendicular
starting_point = np.array([629, -2.8, 52]) #initial position to base
x = np.array([1, 0, 0]) #this input should be calculated by readout real-world position from digitalizer
y = np.array([0, 1, 0])
z = np.cross(x, y)

# Compute the transformed basis
i_prime = (x - starting_point) / np.linalg.norm(x - starting_point)
j_prime = (y - starting_point) / np.linalg.norm(y - starting_point)
if not np.isclose(np.dot(i_prime, j_prime), 0):
    print("not perpendicular")
k_prime = z / np.linalg.norm(z)

# Compute the rotation matrix R, this is a test
R = np.array([
    [np.dot(i, x), np.dot(j, x), np.dot(k, x)],
    [np.dot(i, y), np.dot(j, y), np.dot(k, y)],
    [np.dot(i, z), np.dot(j, z), np.dot(k, z)]
])

# Any point in either coordinate
pinO = np.array([0, 20, 0])

# Transform from global to target coordinate
trans = np.dot(R, (pinO - starting_point))

#from target coordinate system to global, pinOcan be arbitary point in target coordinate 
transB =  np.dot(R.T, pinO) + starting_point 


print(transB)
#Need to be modified and fixed does not accurately rotate model with translational and rotational movement 
def rotation_matrix(axis, theta_degrees): #apply to basis to simulate
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

pitch_test = (math.atan(R[1,2]/R[2,2]))*180/np.pi #pitch = flexion; roll = abduction; yaw = rotation

roll_test = (math.acos(R[0,2])-0.5*np.pi)*180/np.pi
if roll_test > 0 :
    roll_test = 180-roll_test
if roll_test < 0 :
    roll_test = -180-roll_test
yaw_test =  math.atan(R[0,1]/R[0,0])*180/np.pi  #would that be 90 degrees?

adduction = (math.acos(R[0,2])-0.5*np.pi)*180/np.pi #default input as rad
flexion = (math.atan(R[1,2]/R[2,2]))*180/np.pi
rotation = math.atan(R[0,1]/R[0,0])*180/np.pi #change from rad to degree

#test for obtaining rotation matrix
rotation_x = rotation_matrix('x', 30)
rotation_y = rotation_matrix('y',60)
rotation_z = rotation_matrix('z',45)
I = np.eye(3)
rotated_basis = rotation_z @ rotation_y @ rotation_x @ I
print(rotated_basis)

#rotated matrix is got by R = new_basis * original_basis.T
def get_angle(rotated_matrix):
    roll = (math.acos(rotated_matrix[0,2]))*180/np.pi #default input as rad
    pitch = (math.atan(rotated_matrix[1,2]/rotated_matrix[2,2]))*180/np.pi
    yaw = math.atan(rotated_matrix[0,1]/rotated_matrix[0,0])*180/np.pi #change from rad to degree
    return roll,pitch,yaw
print(get_angle(rotated_basis))
roll,pitch,yaw = get_angle(rotated_basis)[0],get_angle(rotated_basis)[1],get_angle(rotated_basis)[2]


transB = np.dot(R, transB - starting_point) + starting_point  # Rotate relative to base , for when you arent using the gripper
#transB_rotated = np.dot(R, transB - starting_point) + starting_point




#test start
from xarm.x3 import XArm, Studio
from xarm.wrapper import XArmAPI

port = '192.168.1.197'

arm = XArmAPI(port)

arm.connect()


arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
arm.set_gripper_position(500)
arm.move_gohome()
arm.set_gripper_enable(1)
arm.set_gripper_position(300)
arm.set_position(x=starting_point[0],y=starting_point[1],z=starting_point[2],roll=-180,pitch=0,yaw=0,speed=50,is_radian=False,wait = True)
arm.set_gripper_position(119)
arm.set_position(x=transB[0], y=transB[1], z=transB[2], roll=180-roll, pitch=pitch, yaw=yaw, speed=50,is_radian=False,wait=True)
#arm.set_position(x=595.8,y=-8.1,z=312.8,roll=-180,pitch=0,yaw=0,speed=50,is_radian=False,wait = True)


print(arm.get_position())
#print("Robot moved to rotated position:", transB_rotated)
arm.disconnect()
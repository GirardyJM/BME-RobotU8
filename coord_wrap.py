import numpy as np
import math
import time
from xarm.x3 import XArm, Studio
from xarm.wrapper import XArmAPI

def initialize_sample_coord_to_global(tibia1,tibia2,femur1,femur2,starting_point,sample_point):
    '''
    This function is used for get ponit in sample space to base
    tibia1 and tibia2 are points in tibia
    femur1 and femur2 are points in femur
    starting_point is position of the sample in base coordinate
    sample_point is the relative position in sample coordinate 
    '''
    tibia1 = np.array(tibia1)
    tibia2 = np.array(tibia2)
    femur1 = np.array(femur1)
    femur2 = np.array(femur2)
    starting_point = np.array(starting_point)
    sample_point = np.array(sample_point)
    # global coordinate in base
    i = np.array([1, 0, 0])
    j = np.array([0, 1, 0])
    k = np.array([0, 0, 1])
    #sample basis

    i_prime = (tibia1 - tibia2) / np.linalg.norm(tibia1-tibia2)
    j_prime = (femur1-femur2) / np.linalg.norm(femur1-femur2)
    z = np.cross(i_prime, j_prime)
    k_prime = z / np.linalg.norm(z)
    # Compute the rotation matrix R
    R = np.array([
        [np.dot(i, i_prime), np.dot(j, i_prime), np.dot(k, i_prime)],
        [np.dot(i, j_prime), np.dot(j, j_prime), np.dot(k, j_prime)],
        [np.dot(i, k_prime), np.dot(j, k_prime), np.dot(k, k_prime)]
    ])

    #from target coordinate system to global 
    transformed_global =  np.dot(R.T, sample_point) + starting_point 
    return np.array(transformed_global)



def move_robot_coord(global_transformed,degrees=np.array([-180,0,0]),speed=20):
    '''
    This function is to move to target position after transformation
    global_transformed is the point in sample transformed to global
    degrees is from rotational matrix
    '''
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    x,y,z = global_transformed
    pitch,yaw,roll = degrees
    position = np.array([x,y,z,pitch,yaw,roll])
    arm.set_position(x,y,z,pitch,yaw,roll,radius=False,wait=False,speed=speed)
    arm.disconnect()
    return position

def get_rotational_matrix(tibia1,tibia2,femur1,femur2,starting_point,sample_point):
    '''
    This fuction is to get rotational matrix from given coordinates
    tibia1 and tibia2 are points in tibia
    femur1 and femur2 are points in femur
    starting_point is position of the sample in base coordinate
    sample_point is the relative position in sample coordinate 
    '''
    tibia1 = np.array(tibia1)
    tibia2 = np.array(tibia2)
    femur1 = np.array(femur1)
    femur2 = np.array(femur2)
    starting_point = np.array(starting_point)
    sample_point = np.array(sample_point)
    # global coordinate in base
    i = np.array([1, 0, 0])
    j = np.array([0, 1, 0])
    k = np.array([0, 0, 1])
    #sample basis
    k_prime = (tibia1 - tibia2) / np.linalg.norm(tibia1-tibia2)
    j_prime = (femur1-femur2) / np.linalg.norm(femur1-femur2)
    z = np.cross(k_prime, j_prime)
    i_prime = z / np.linalg.norm(z)
    # Compute the rotation matrix R
    R = np.array([
        [np.dot(i, i_prime), np.dot(j, i_prime), np.dot(k, i_prime)],
        [np.dot(i, j_prime), np.dot(j, j_prime), np.dot(k, j_prime)],
        [np.dot(i, k_prime), np.dot(j, k_prime), np.dot(k, k_prime)]
    ])
    row = (math.acos(R[0,2])-0.5*np.pi)*180/np.pi #default input as rad
    if row <0:
        row = -180-row
    else:
        row = 180-row
    pitch = (math.atan(R[1,2]/R[2,2]))*180/np.pi
    yaw = math.atan(R[0,1]/R[0,0])*180/np.pi #change from rad to degree
    degrees = np.array([row,pitch,yaw])
    return degrees

def move_robot_arm(position,degrees=np.array([-180,0,0]),speed=20):  
#this function is suppose to move the robot arm to the sample
    x,y,z = position
    pitch,yaw,roll = degrees
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    arm.set_position(x,y,z,pitch,yaw,roll,False,speed,wait=True)
    arm.disconnect()
    return

def get_robot_position():
    #This function is to get current position of the robot
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    position = arm.get_position(is_radian=False)
    if position[0]!=0:
        position[1]=0
    position = np.asarray(position[1])
    arm.disconnect()
    return position

def go_home():
    #This function is to set robot to initial position
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    arm.move_gohome(wait=True)
    arm.disconnect()
    return

def calculate_length(sample_position,current_position):
#this function is to calculate the length of sample point to tool center point
    x1,y1,z1 = sample_position
    x2,y2,z2 = current_position[0:3]
    length = ((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)**0.5
    return length

def set_tool_position(degrees):
    #after initialization, use set tool position for modification and movement
    pitch,yaw,roll = degrees
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    arm.set_tool_position(0,0,0,pitch=pitch,yaw=yaw,roll=roll,is_radian=False)
    arm.disconnect()
    return
def set_position(position):
    x,y,z = np.array(position)
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    arm.set_position(x,y,z,180,0,0,is_radian=False,wait=True)
    arm.disconnect()
    return

def read_force(): #display delay is about 1.5s
    ip = '192.168.1.197'
    arm = XArmAPI(ip, enable_report=True)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.ft_sensor_enable(0)
    #arm.clean_error()
    #arm.clean_warn()
    arm.ft_sensor_enable(1)
    time.sleep(0.02)
    #arm.ft_sensor_set_zero()
    ft_ext=arm.get_ft_sensor_data()
    time.sleep(0.01)
    arm.ft_sensor_enable(0)
    arm.disconnect()
    return np.array(ft_ext[1])
'''
tibia1 = np.array([1,0,0])
tibia2 = np.array([2,0,0])
femur1 = np.array([0,1,0])
femur2 = np.array([0,2,0])
starting_point = np.array([200,20,25])
sample_point = np.array([0,0,20])
test = initialize_sample_coord_to_global(tibia1,tibia2,femur1,femur2,starting_point,sample_point)
degrees = get_rotational_matrix(tibia1,tibia2,femur1,femur2,starting_point,sample_point)
position = move_robot_coord(test,degrees)
print(position)

t1 = time.time()
read_force()
t2 = time.time()
t = (t2-t1)*1000
print("whole calling time")
print(t)
ip = '192.168.1.197'
arm = XArmAPI(ip, enable_report=True)
arm.connect()
arm.motion_enable(enable=True)
arm.ft_sensor_enable(0)
arm.clean_error()
arm.clean_warn()
arm.ft_sensor_enable(1)
time.sleep(0.02)
arm.ft_sensor_set_zero()
t1 = time.time()
ft_ext=arm.get_ft_sensor_data()
t2 = time.time()
t = t2-t1
time.sleep(0.01)
arm.ft_sensor_enable(0)
arm.disconnect()
print("only exectute get function time")
print(t*1000)
'''

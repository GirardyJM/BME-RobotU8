import numpy as np
import math
from xarm.x3 import XArm, Studio
from xarm.wrapper import XArmAPI
'''Function Documentation
external_rotation_step(arm, position_map, increment=1, current_ie_angle=0)
Inputs:

arm: XArmAPI object for controlling the robot arm
position_map: Dictionary mapping angles to position arrays [x, y, z, roll, pitch, yaw]
increment: Float representing the increment to increase external rotation in degrees (default: 1)
current_ie_angle: Float representing the current internal/external rotation angle in degrees (default: 0)

set_internal_external_rotation(arm, position_map, target_ie_angle)
Inputs:

position_map: Dictionary mapping angles to position arrays [x, y, z, roll, pitch, yaw]
target_ie_angle: Float representing the target internal/external rotation angle in degrees (-30 to 30, positive for internal, negative for external)'''

def create_transformation_matrix(starting_point):
    """Creates and returns the transformation matrix for knee-to-robot coordinates
    starting_point: 2D numpy array representing the starting position of the robot [x, y, z]"""
    point_spacing = 25.4
    
    p1 = starting_point + np.array([-point_spacing / 2, 0, 0])
    p2 = starting_point
    p3 = starting_point + np.array([point_spacing / 2, 0, 0])
    
    O_robot = p2
    x_robot = p3 - p1
    y_robot = np.array([0, 1, 0])
    z_robot = np.cross(x_robot, y_robot)
    
    x_robot = x_robot / np.linalg.norm(x_robot)
    y_robot = y_robot / np.linalg.norm(y_robot)
    z_robot = z_robot / np.linalg.norm(z_robot)
    
    A = np.array([620.4453110694885, 47.15324938297272, 171.99364304542542])
    B = np.array([626.9657611846924, -29.983650892972946, 171.94780707359314])
    C = np.array([626.97, 3.73791, 88.1255])
    D = np.array([626.97, 3.73, 40])
    
    Of = (A + B) / 2
    Ot = Of + (D - C)
    x_knee = Of - Ot
    y_knee = A - B
    z_knee = np.cross(x_knee, y_knee)
    
    x_knee = x_knee / np.linalg.norm(x_knee)
    y_knee = y_knee / np.linalg.norm(y_knee)
    z_knee = z_knee / np.linalg.norm(z_knee)
    
    R_knee_to_robot = np.array([x_robot, y_robot, z_robot]).T @ np.array([x_knee, y_knee, z_knee])
    translation = O_robot - Of
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R_knee_to_robot
    transformation_matrix[:3, 3] = translation
    
    return transformation_matrix, Of

def calculate_gs_angles(transformation_matrix):
    """
    Calculate Grood-Suntay angles from transformation matrix
    transformation_matrix: 4x4 numpy array representing the transformation matrix between knee and robot coordinates
    """
    Tft = transformation_matrix[:3, :3]
    
    Fx = np.array([1, 0, 0])
    Fy = np.array([0, 1, 0])
    
    Tft_x = Tft[:, 0]
    Tft_y = Tft[:, 1]
    
    e2 = np.cross(Tft_x, Fy)
    e2_norm = np.linalg.norm(e2)
    e2_unit = e2 / e2_norm
    
    output = np.cross(e2_unit, Fx)
    if output[1] > 0:
        alpha = np.arcsin(np.dot(e2_unit, Fx)) * 180/np.pi
    else:
        alpha = -180 - np.arcsin(np.dot(e2_unit, Fx)) * 180/np.pi
    
    beta = 90 - np.arccos(np.dot(Fy, Tft_x)) * 180/np.pi
    gamma = np.arcsin(np.dot(e2_unit, Tft_y)) * 180/np.pi
    
    return np.array([gamma, alpha, beta])

def create_gs_rotation_matrix(ie_angle, fe_angle, vv_angle):
    """
    Create rotation matrix from Grood-Suntay angles
    ie_angle: Float representing internal/external rotation angle in degrees
    fe_angle: Float representing flexion/extension angle in degrees
    vv_angle: Float representing varus/valgus angle in degrees
    """
    ie = np.radians(ie_angle)
    fe = np.radians(fe_angle)
    vv = np.radians(vv_angle)
    
    #calculate rotation matrix for each axis
    R_fe = np.array([
        [np.cos(fe), -np.sin(fe), 0],
        [np.sin(fe), np.cos(fe), 0],
        [0, 0, 1]
    ])
    R_ie = np.array([
        [1, 0, 0],
        [0, np.cos(ie), -np.sin(ie)],
        [0, np.sin(ie), np.cos(ie)]
    ])
    R_vv = np.array([
        [np.cos(vv), 0, -np.sin(vv)],
        [0, 1, 0],
        [np.sin(vv), 0, np.cos(vv)]
    ])
    
    return R_vv @ R_fe @ R_ie

def calculate_gs_position(fe_angle, ie_angle=0, vv_angle=0, starting_point=None):
    """
    Calculate new position using Grood-Suntay angles
    fe_angle: Float representing flexion/extension angle in degrees
    ie_angle: Float representing internal/external rotation angle in degrees 
    vv_angle: Float representing varus/valgus angle in degrees 
    starting_point: 3D numpy array representing the starting position [x, y, z] 
    """
    if starting_point is None: #defalt testing value
        starting_point = np.array([597, -31.4, 317.7])
    
    transformation_matrix, Of = create_transformation_matrix(starting_point)
    rotation = create_gs_rotation_matrix(ie_angle, fe_angle, vv_angle)
    Of_homog = np.append(Of, 1)
    pivot_knee = np.linalg.inv(transformation_matrix) @ Of_homog
    
    head_homog = np.append(starting_point, 1)
    head_knee = np.linalg.inv(transformation_matrix) @ head_homog
    
    pivot_to_head = head_knee[:3] - pivot_knee[:3]
    rotated_vector = rotation @ pivot_to_head
    new_head_knee = pivot_knee[:3] + rotated_vector
    
    new_head_homog = np.append(new_head_knee, 1)
    new_head_global = transformation_matrix @ new_head_homog

    return new_head_global[:3]

def initialize_flexion_extension_control(starting_point=None):
    """
    Initialize the flexion/extension system and pre-calculate positions
    starting_point: 2D numpy array representing the starting position of the robot [x, y, z]
    """
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])
    
    #transformation_matrix, Of = create_transformation_matrix(starting_point)
    
    angle_range = list(range(0, 121))
    position_map = {} #This is dictionary!
    
    for angle in angle_range:
        pos = calculate_gs_position(angle, 0, 0, starting_point)
        roll_val = -180 + angle
        position_map[angle] = [
            float(pos[0]), float(pos[1]), float(pos[2]), 
            roll_val, 0, 0
        ]
    
    arm.set_position(x=starting_point[0],y=starting_point[1],z=starting_point[2],
        roll=-180,pitch=0,yaw=0,speed=50,wait=True
    )
    arm.disconnect()
    
    return position_map #not sure whether it will work

def flexion_step_control(angle_increment=1, starting_point=None, current_angle=0):
    """
    Increase flexion by specified increment
    angle_increment: Float representing the increment to increase flexion angle in degrees
    starting_point: 3D numpy array representing the starting position [x, y, z] 
    current_angle: Float representing the current flexion angle in degrees 
    """
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])
    
    position_map = initialize_flexion_extension_control(starting_point)
    
    new_angle = current_angle + angle_increment
    
    if new_angle > 120:
        arm.disconnect()
        return np.array([current_angle,0])
    
    target_angle_int = int(round(new_angle)) #are you sure return angles that are round to int?
    target_position = position_map[target_angle_int]
    
    try:
        arm.set_position(
            x=target_position[0],y=target_position[1],z=target_position[2],
            roll=target_position[3],pitch=target_position[4],yaw=target_position[5],
            speed=30,wait=True
        )
        
        current_angle = target_angle_int
        arm.disconnect()
        return np.array([current_angle, 1])  #1 means the result is correct while 0 means not
        
    except Exception as e:
        arm.disconnect()
        return np.array([current_angle,0])

def extension_step_control(angle_increment=1, starting_point=None, current_angle=0):
    """
    Increase extension by specified increment
    angle_increment: Float representing the increment to increase extension angle in degrees 
    starting_point: 3D numpy array representing the starting position [x, y, z] 
    current_angle: Float representing the current flexion angle in degrees
    """
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])
    
    position_map = initialize_flexion_extension_control(starting_point)
    
    new_angle = current_angle - angle_increment
    
    if new_angle < 0:
        arm.disconnect()
        return np.array([current_angle,0])
    
    target_angle_int = int(round(new_angle)) #same problem
    target_position = position_map[target_angle_int]
    
    try:
        arm.set_position(x=target_position[0],y=target_position[1],z=target_position[2],
            roll=target_position[3],pitch=target_position[4],yaw=target_position[5],
            speed=30,wait=True)
        
        current_angle = target_angle_int
        arm.disconnect()
        return np.array([current_angle,1])
        
    except Exception as e:
        arm.disconnect()
        return np.array([current_angle,0])

def set_specific_flexion_angle(target_angle, starting_point=None):
    """
    Move to a specific flexion angle   
    target_angle: Float representing the target flexion angle in degrees (0-120)
    starting_point: numpy array representing the starting position [x, y, z] 
    """
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])
    
    position_map = initialize_flexion_extension_control(starting_point)
    
    if target_angle < 0:
        arm.disconnect()
        return np.array([0,0])
    
    if target_angle > 120:
        target_angle = 120
    
    target_angle_int = int(round(target_angle))
    target_position = position_map[target_angle_int]
    
    try:
        arm.set_position(x=target_position[0],y=target_position[1],z=target_position[2],
            roll=target_position[3],pitch=target_position[4],yaw=target_position[5],
            speed=30,wait=True)
        
        arm.disconnect()
        return np.array([target_angle_int, 1])
        
    except Exception as e:
        arm.disconnect()
        return np.array([0,0])

def reset_to_start_position(starting_point=None):
    """Return the robot to the starting position
    starting_point: numpy array representing the starting position [x, y, z] """
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])
    
    try:
        arm.set_position(x=starting_point[0],y=starting_point[1],z=starting_point[2],
            roll=-180,pitch=0,yaw=0,
            speed=50,wait=True)
        
        arm.disconnect()
        return 1
        
    except Exception as e:
        arm.disconnect()
        return 0
    

def internal_rotation_step(position_map, increment=1, current_ie_angle=0):
    '''
    position_map: Dictionary mapping angles to position arrays [x, y, z, roll, pitch, yaw]
    increment: Float representing the increment to increase internal rotation in degrees (default: 1)
    current_ie_angle: Float representing the current internal/external rotation angle in degrees (default: 0)
    '''
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)

    global current_knee_angle #be careful with global
    
    new_ie_angle = current_ie_angle + increment
    
    if new_ie_angle > 30:
        arm.disconnect()
        return np.array([current_ie_angle, 0]) #I think here is false rather than true since it is hitting maxium
    
    try:
        target_angle_int = int(round(current_knee_angle))
        target_position = position_map[target_angle_int].copy()
        
        target_position[5] = float(new_ie_angle)
        
        
        arm.set_position(x=target_position[0],y=target_position[1],z=target_position[2],
            roll=target_position[3],pitch=target_position[4],yaw=target_position[5],
            speed=30,wait=True
        )
        arm.disconnect()
        return np.array([new_ie_angle, 1])
        
    except Exception as e:
        arm.disconnect()
        return np.array([current_ie_angle, 0])

def external_rotation_step(position_map, increment=1, current_ie_angle=0):
    '''
    position_map: Dictionary mapping angles to position arrays [x, y, z, roll, pitch, yaw]
    increment: Float representing the increment to increase external rotation in degrees (default: 1)
    current_ie_angle: Float representing the current internal/external rotation angle in degrees (default: 0)
    '''
    global current_knee_angle
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)

    new_ie_angle = current_ie_angle - increment
    
    if new_ie_angle < -30:
        arm.disconnect()
        return np.array([current_ie_angle, 0]) 
    
    try:
        target_angle_int = int(round(current_knee_angle))
        target_position = position_map[target_angle_int].copy()
        target_position[5] = float(new_ie_angle)

        arm.set_position(x=target_position[0],y=target_position[1],z=target_position[2],
            roll=target_position[3],pitch=target_position[4],yaw=target_position[5],
            speed=30,wait=True)
        
        return np.array([new_ie_angle, 1])
        
    except Exception as e:
        return np.array([current_ie_angle, 0])

def set_internal_external_rotation(position_map, target_ie_angle): #labview cannot handle dictionary
    '''
    position_map: Dictionary mapping angles to position arrays [x, y, z, roll, pitch, yaw]
    target_ie_angle: Float representing the target internal/external rotation angle in degrees (-30 to 30, positive for internal, negative for external)
    '''
    global current_knee_angle
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    if target_ie_angle > 30:
        #Maximum internal rotation is 30 degrees. Setting to 30
        target_ie_angle = 30
    elif target_ie_angle < -30:
        #Maximum external rotation is 30 degrees. Setting to -30
        target_ie_angle = -30
    
    try:
        target_angle_int = int(round(current_knee_angle))
        target_position = position_map[target_angle_int].copy()
        
        target_position[5] = float(target_ie_angle)
        
        #rotation_type = "internal" if target_ie_angle >= 0 else "external"
        #display_angle = abs(target_ie_angle)
        #print(f"Moving to {display_angle} degrees {rotation_type} rotation")
        #print(f"Current flexion angle: {current_knee_angle} degrees")
        
        arm.set_position(
            x=target_position[0],y=target_position[1],z=target_position[2],
            roll=target_position[3],pitch=target_position[4],yaw=target_position[5],
            speed=30,wait=True)
        arm.disconnect()
        return np.array([target_ie_angle, 1])
        
    except Exception as e:
        arm.disconnect()
        return np.array([0, 0])

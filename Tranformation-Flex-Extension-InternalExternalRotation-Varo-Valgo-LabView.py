'''Function Documentation
create_transformation_matrix(starting_point)
Inputs:

starting_point: 3D numpy array representing the starting position of the robot [x, y, z]

calculate_gs_angles(transformation_matrix)
Inputs:

transformation_matrix: 4x4 numpy array representing the transformation matrix between knee and robot coordinates

create_gs_rotation_matrix(ie_angle, fe_angle, vv_angle)
Inputs:

ie_angle: Float representing internal/external rotation angle in degrees
fe_angle: Float representing flexion/extension angle in degrees
vv_angle: Float representing varus/valgus angle in degrees

calculate_gs_position(fe_angle, ie_angle=0, vv_angle=0, starting_point=None)
Inputs:

fe_angle: Float representing flexion/extension angle in degrees
ie_angle: Float representing internal/external rotation angle in degrees (default: 0)
vv_angle: Float representing varus/valgus angle in degrees (default: 0)
starting_point: 3D numpy array representing the starting position [x, y, z] (default: [597, -31.4, 317.7])

initialize_flexion_extension_control(starting_point=None)
Inputs:

starting_point: 3D numpy array representing the starting position [x, y, z] (default: [597, -31.4, 317.7])

flexion_step_control(angle_increment=1, starting_point=None, current_angle=0)
Inputs:

angle_increment: Float representing the increment to increase flexion angle in degrees (default: 1)
starting_point: 3D numpy array representing the starting position [x, y, z] (default: [597, -31.4, 317.7])
current_angle: Float representing the current flexion angle in degrees (default: 0)

extension_step_control(angle_increment=1, starting_point=None, current_angle=0)
Inputs:

angle_increment: Float representing the increment to increase extension angle in degrees (default: 1)
starting_point: 3D numpy array representing the starting position [x, y, z] (default: [597, -31.4, 317.7])
current_angle: Float representing the current flexion angle in degrees (default: 0)

set_specific_flexion_angle(target_angle, starting_point=None)
Inputs:

target_angle: Float representing the target flexion angle in degrees (0-120)
starting_point: 3D numpy array representing the starting position [x, y, z] (default: [597, -31.4, 317.7])

reset_to_start_position(starting_point=None)
Inputs:

starting_point: 3D numpy array representing the starting position [x, y, z] (default: [597, -31.4, 317.7])

internal_rotation_step(arm, position_map, increment=1, current_ie_angle=0)
Inputs:

arm: XArmAPI object for controlling the robot arm
position_map: Dictionary mapping angles to position arrays [x, y, z, roll, pitch, yaw]
increment: Float representing the increment to increase internal rotation in degrees (default: 1)
current_ie_angle: Float representing the current internal/external rotation angle in degrees (default: 0)

external_rotation_step(arm, position_map, increment=1, current_ie_angle=0)
Inputs:

arm: XArmAPI object for controlling the robot arm
position_map: Dictionary mapping angles to position arrays [x, y, z, roll, pitch, yaw]
increment: Float representing the increment to increase external rotation in degrees (default: 1)
current_ie_angle: Float representing the current internal/external rotation angle in degrees (default: 0)

set_internal_external_rotation(arm, position_map, target_ie_angle)
Inputs:

arm: XArmAPI object for controlling the robot arm
position_map: Dictionary mapping angles to position arrays [x, y, z, roll, pitch, yaw]
target_ie_angle: Float representing the target internal/external rotation angle in degrees (-30 to 30, positive for internal, negative for external)'''

import numpy as np
import sys
import os
import time
import math
from xarm.x3 import XArm, Studio
from xarm.wrapper import XArmAPI


# Global variable to track the current knee angle across functions
current_knee_angle = 0

def create_transformation_matrix(starting_point):
    """Creates and returns the transformation matrix for knee-to-robot coordinates"""
    point_spacing = 25.4  # Setting the point spacing to 25.4mm (1 inch)
    
    p1 = starting_point + np.array([-point_spacing / 2, 0, 0])  # Creating point p1 by moving half a spacing left from starting point
    p2 = starting_point  # Setting p2 as the starting point
    p3 = starting_point + np.array([point_spacing / 2, 0, 0])  # Creating point p3 by moving half a spacing right from starting point
    
    O_robot = p2  # Setting robot origin to p2 (the middle point)
    x_robot = p3 - p1  # Creating robot x-axis as vector from p1 to p3
    y_robot = np.array([0, 1, 0])  # Setting robot y-axis as upward unit vector
    z_robot = np.cross(x_robot, y_robot)  # Creating robot z-axis as cross product of x and y axes
    
    x_robot = x_robot / np.linalg.norm(x_robot)  # Normalizing x-axis to unit length
    y_robot = y_robot / np.linalg.norm(y_robot)  # Normalizing y-axis to unit length
    z_robot = z_robot / np.linalg.norm(z_robot)  # Normalizing z-axis to unit length
    
    # Setting hard-coded anatomical reference points for knee coordinate system
    A = np.array([620.4453110694885, 47.15324938297272, 171.99364304542542])  # Point A on knee
    B = np.array([626.9657611846924, -29.983650892972946, 171.94780707359314])  # Point B on knee
    C = np.array([626.97, 3.73791, 88.1255])  # Point C on knee
    D = np.array([626.97, 3.73, 40])  # Point D on knee
    
    Of = (A + B) / 2  # Setting femur origin as midpoint of A and B
    Ot = Of + (D - C)  # Setting tibia origin by offsetting femur origin
    x_knee = Of - Ot  # Creating knee x-axis as vector from tibia origin to femur origin
    y_knee = A - B  # Creating knee y-axis as vector from B to A
    z_knee = np.cross(x_knee, y_knee)  # Creating knee z-axis as cross product of x and y axes
    
    x_knee = x_knee / np.linalg.norm(x_knee)  # Normalizing knee x-axis to unit length
    y_knee = y_knee / np.linalg.norm(y_knee)  # Normalizing knee y-axis to unit length
    z_knee = z_knee / np.linalg.norm(z_knee)  # Normalizing knee z-axis to unit length
    
    # Creating the rotation matrix from knee coordinates to robot coordinates
    R_knee_to_robot = np.array([x_robot, y_robot, z_robot]).T @ np.array([x_knee, y_knee, z_knee])
    
    translation = O_robot - Of  # Calculating translation vector from femur origin to robot origin
    
    # Creating the full 4x4 transformation matrix
    transformation_matrix = np.eye(4)  # Starting with 4x4 identity matrix
    transformation_matrix[:3, :3] = R_knee_to_robot  # Setting upper 3x3 as rotation matrix
    transformation_matrix[:3, 3] = translation  # Setting rightmost column as translation vector
    
    return transformation_matrix, Of  # Returning the transformation matrix and femur origin

def calculate_gs_angles(transformation_matrix):
    """Calculate Grood-Suntay angles from transformation matrix"""
    Tft = transformation_matrix[:3, :3]  # Extracting rotation matrix from transformation matrix
    
    Fx = np.array([1, 0, 0])  # Defining femur x-axis unit vector
    Fy = np.array([0, 1, 0])  # Defining femur y-axis unit vector
    
    Tft_x = Tft[:, 0]  # First column of rotation matrix (tibia x-axis in femur coordinates)
    Tft_y = Tft[:, 1]  # Second column of rotation matrix (tibia y-axis in femur coordinates)
    
    e2 = np.cross(Tft_x, Fy)  # Computing floating axis as cross product of tibia x-axis and femur y-axis
    e2_norm = np.linalg.norm(e2)  # Calculating magnitude of floating axis
    e2_unit = e2 / e2_norm  # Normalizing floating axis to unit length
    
    output = np.cross(e2_unit, Fx)  # Cross product used for angle determination
    
    # Calculating alpha (flexion/extension angle) with quadrant check
    if output[1] > 0:
        alpha = np.arcsin(np.dot(e2_unit, Fx)) * 180/np.pi  # Converting from radians to degrees
    else:
        alpha = -180 - np.arcsin(np.dot(e2_unit, Fx)) * 180/np.pi  # Adjusting for different quadrant
    
    # Calculating beta (varus/valgus angle)
    beta = 90 - np.arccos(np.dot(Fy, Tft_x)) * 180/np.pi
    
    # Calculating gamma (internal/external rotation angle)
    gamma = np.arcsin(np.dot(e2_unit, Tft_y)) * 180/np.pi
    
    return np.array([gamma, alpha, beta])  # Returning the three Grood-Suntay angles as numpy array

def create_gs_rotation_matrix(ie_angle, fe_angle, vv_angle):
    """Create rotation matrix from Grood-Suntay angles"""
    ie = np.radians(ie_angle)  # Converting internal/external rotation angle to radians
    fe = np.radians(fe_angle)  # Converting flexion/extension angle to radians
    vv = np.radians(vv_angle)  # Converting varus/valgus angle to radians
    
    # Creating rotation matrix for flexion/extension around y-axis
    R_fe = np.array([
        [np.cos(fe), -np.sin(fe), 0],
        [np.sin(fe), np.cos(fe), 0],
        [0, 0, 1]
    ])
    
    # Creating rotation matrix for internal/external rotation around x-axis
    R_ie = np.array([
        [1, 0, 0],
        [0, np.cos(ie), -np.sin(ie)],
        [0, np.sin(ie), np.cos(ie)]
    ])
    
    # Creating rotation matrix for varus/valgus around z-axis
    R_vv = np.array([
        [np.cos(vv), 0, -np.sin(vv)],
        [0, 1, 0],
        [np.sin(vv), 0, np.cos(vv)]
    ])
    
    # Returning the combined rotation matrix by multiplying the individual rotation matrices
    return R_vv @ R_fe @ R_ie

def calculate_gs_position(fe_angle, ie_angle=0, vv_angle=0, starting_point=None):
    """Calculate new position using Grood-Suntay angles"""
    port = '192.168.1.197'  # Setting the IP address of the robot arm
    arm = XArmAPI(port)  # Creating XArmAPI object for controlling the robot arm
    arm.connect()  # Connecting to the robot arm
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])  # Using default starting point if none provided
    
    # Creating transformation matrix and getting femur origin
    transformation_matrix, Of = create_transformation_matrix(starting_point)
    
    # Creating rotation matrix from Grood-Suntay angles
    rotation = create_gs_rotation_matrix(ie_angle, fe_angle, vv_angle)
    
    # Converting femur origin to homogeneous coordinates (adding 1 at the end)
    Of_homog = np.append(Of, 1)
    
    # Transforming femur origin to knee coordinates
    pivot_knee = np.linalg.inv(transformation_matrix) @ Of_homog
    
    # Converting starting point to homogeneous coordinates
    head_homog = np.append(starting_point, 1)
    
    # Transforming starting point to knee coordinates
    head_knee = np.linalg.inv(transformation_matrix) @ head_homog
    
    # Calculating vector from pivot to head in knee coordinates
    pivot_to_head = head_knee[:3] - pivot_knee[:3]
    
    # Rotating the vector using Grood-Suntay rotation matrix
    rotated_vector = rotation @ pivot_to_head
    
    # Calculating new head position in knee coordinates
    new_head_knee = pivot_knee[:3] + rotated_vector
    
    # Converting new head position to homogeneous coordinates
    new_head_homog = np.append(new_head_knee, 1)
    
    # Transforming new head position back to global coordinates
    new_head_global = transformation_matrix @ new_head_homog
    
    arm.disconnect()  # Disconnecting from the robot arm
    
    return new_head_global[:3]  # Returning the new position in global coordinates as numpy array

def initialize_flexion_extension_control(starting_point=None):
    """Initialize the flexion/extension system and pre-calculate positions"""
    port = '192.168.1.197'  # Setting the IP address of the robot arm
    arm = XArmAPI(port)  # Creating XArmAPI object for controlling the robot arm
    arm.connect()  # Connecting to the robot arm
    arm.motion_enable(enable=True)  # Enabling motion on the robot arm
    arm.set_mode(0)  # Setting robot to position control mode
    arm.set_state(state=0)  # Setting robot to ready state
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])  # Using default starting point if none provided
    
    # Creating transformation matrix and getting femur origin
    transformation_matrix, Of = create_transformation_matrix(starting_point)
    
    angle_range = list(range(0, 121))  # Creating a range of angles from 0 to 120 degrees
    position_map = {}  # Initializing empty dictionary to store position data for each angle
    
    # Pre-calculating position for each angle in the range
    for angle in angle_range:
        # Calculating position for this angle with no IE or VV rotation
        pos = calculate_gs_position(angle, 0, 0, starting_point)
        roll_val = -180 + angle  # Calculating roll value for robot (changes with flexion angle)
        
        # Storing position data as [x, y, z, roll, pitch, yaw]
        position_map[angle] = [
            float(pos[0]), float(pos[1]), float(pos[2]), 
            roll_val, 0, 0
        ]
    
    # Moving robot to starting position
    arm.set_position(
        x=starting_point[0],
        y=starting_point[1],
        z=starting_point[2],
        roll=-180,  # Starting roll value for full extension
        pitch=0,
        yaw=0,
        speed=50,
        wait=True  # Wait until movement is complete
    )
    
    arm.disconnect()  # Disconnecting from the robot arm
    
    return position_map  # Returning the map of angles to positions

def flexion_step_control(angle_increment=1, starting_point=None, current_angle=0):
    """Increase flexion by specified increment"""
    global current_knee_angle  # Using global variable to track knee angle
    
    port = '192.168.1.197'  # Setting the IP address of the robot arm
    arm = XArmAPI(port)  # Creating XArmAPI object for controlling the robot arm
    arm.connect()  # Connecting to the robot arm
    arm.motion_enable(enable=True)  # Enabling motion on the robot arm
    arm.set_mode(0)  # Setting robot to position control mode
    arm.set_state(state=0)  # Setting robot to ready state
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])  # Using default starting point if none provided
    
    # Initializing and getting position map
    position_map = initialize_flexion_extension_control(starting_point)
    
    new_angle = current_angle + angle_increment  # Calculating new angle after increment
    
    if new_angle > 120:  # Checking if new angle exceeds maximum
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([current_angle, 0])  # Returning current angle and failure flag as numpy array
    
    target_angle_int = int(round(new_angle))  # Rounding new angle to nearest integer
    target_position = position_map[target_angle_int]  # Getting pre-calculated position for this angle
    
    try:
        # Moving robot to target position
        arm.set_position(
            x=target_position[0],
            y=target_position[1],
            z=target_position[2],
            roll=target_position[3],
            pitch=target_position[4],
            yaw=target_position[5],
            speed=30,
            wait=True  # Wait until movement is complete
        )
        
        current_angle = target_angle_int  # Updating current angle
        current_knee_angle = current_angle  # Updating global knee angle variable
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([current_angle, 1])  # Returning new angle and success flag as numpy array
        
    except Exception as e:
        arm.disconnect()  # Disconnecting from the robot arm on error
        return np.array([current_angle, 0])  # Returning current angle and failure flag as numpy array

def extension_step_control(angle_increment=1, starting_point=None, current_angle=0):
    """Increase extension by specified increment"""
    global current_knee_angle  # Using global variable to track knee angle
    
    port = '192.168.1.197'  # Setting the IP address of the robot arm
    arm = XArmAPI(port)  # Creating XArmAPI object for controlling the robot arm
    arm.connect()  # Connecting to the robot arm
    arm.motion_enable(enable=True)  # Enabling motion on the robot arm
    arm.set_mode(0)  # Setting robot to position control mode
    arm.set_state(state=0)  # Setting robot to ready state
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])  # Using default starting point if none provided
    
    # Initializing and getting position map
    position_map = initialize_flexion_extension_control(starting_point)
    
    new_angle = current_angle - angle_increment  # Calculating new angle after decrement (extension is reducing angle)
    
    if new_angle < 0:  # Checking if new angle is below minimum
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([current_angle, 0])  # Returning current angle and failure flag as numpy array
    
    target_angle_int = int(round(new_angle))  # Rounding new angle to nearest integer
    target_position = position_map[target_angle_int]  # Getting pre-calculated position for this angle
    
    try:
        # Moving robot to target position
        arm.set_position(
            x=target_position[0],
            y=target_position[1],
            z=target_position[2],
            roll=target_position[3],
            pitch=target_position[4],
            yaw=target_position[5],
            speed=30,
            wait=True  # Wait until movement is complete
        )
        
        current_angle = target_angle_int  # Updating current angle
        current_knee_angle = current_angle  # Updating global knee angle variable
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([current_angle, 1])  # Returning new angle and success flag as numpy array
        
    except Exception as e:
        arm.disconnect()  # Disconnecting from the robot arm on error
        return np.array([current_angle, 0])  # Returning current angle and failure flag as numpy array

def set_specific_flexion_angle(target_angle, starting_point=None):
    """Move to a specific flexion angle"""
    global current_knee_angle  # Using global variable to track knee angle
    
    port = '192.168.1.197'  # Setting the IP address of the robot arm
    arm = XArmAPI(port)  # Creating XArmAPI object for controlling the robot arm
    arm.connect()  # Connecting to the robot arm
    arm.motion_enable(enable=True)  # Enabling motion on the robot arm
    arm.set_mode(0)  # Setting robot to position control mode
    arm.set_state(state=0)  # Setting robot to ready state
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])  # Using default starting point if none provided
    
    # Initializing and getting position map
    position_map = initialize_flexion_extension_control(starting_point)
    
    if target_angle < 0:  # Checking if target angle is below minimum
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([0, 0])  # Returning 0 angle and failure flag as numpy array
    
    if target_angle > 120:  # Capping target angle at maximum
        target_angle = 120
    
    target_angle_int = int(round(target_angle))  # Rounding target angle to nearest integer
    target_position = position_map[target_angle_int]  # Getting pre-calculated position for this angle
    
    try:
        # Moving robot to target position
        arm.set_position(
            x=target_position[0],
            y=target_position[1],
            z=target_position[2],
            roll=target_position[3],
            pitch=target_position[4],
            yaw=target_position[5],
            speed=30,
            wait=True  # Wait until movement is complete
        )
        
        current_knee_angle = target_angle_int  # Updating global knee angle variable
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([target_angle_int, 1])  # Returning target angle and success flag as numpy array
        
    except Exception as e:
        arm.disconnect()  # Disconnecting from the robot arm on error
        return np.array([0, 0])  # Returning 0 angle and failure flag as numpy array

def reset_to_start_position(starting_point=None):
    """Return the robot to the starting position"""
    global current_knee_angle  # Using global variable to track knee angle
    
    port = '192.168.1.197'  # Setting the IP address of the robot arm
    arm = XArmAPI(port)  # Creating XArmAPI object for controlling the robot arm
    arm.connect()  # Connecting to the robot arm
    arm.motion_enable(enable=True)  # Enabling motion on the robot arm
    arm.set_mode(0)  # Setting robot to position control mode
    arm.set_state(state=0)  # Setting robot to ready state
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])  # Using default starting point if none provided
    
    try:
        # Moving robot back to starting position
        arm.set_position(
            x=starting_point[0],
            y=starting_point[1],
            z=starting_point[2],
            roll=-180,  # Starting roll value for full extension
            pitch=0,
            yaw=0,
            speed=50,
            wait=True  # Wait until movement is complete
        )
        
        current_knee_angle = 0  # Resetting global knee angle to 0 (full extension)
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([1])  # Returning success flag as numpy array
        
    except Exception as e:
        arm.disconnect()  # Disconnecting from the robot arm on error
        return np.array([0])  # Returning failure flag as numpy array

def internal_rotation_step(increment=1, current_ie_angle=0, starting_point=None):
    """Increase internal rotation by specified increment"""
    global current_knee_angle  # Using global variable for current knee flexion angle
    
    port = '192.168.1.197'  # Setting the IP address of the robot arm
    arm = XArmAPI(port)  # Creating XArmAPI object for controlling the robot arm
    arm.connect()  # Connecting to the robot arm
    arm.motion_enable(enable=True)  # Enabling motion on the robot arm
    arm.set_mode(0)  # Setting robot to position control mode
    arm.set_state(state=0)  # Setting robot to ready state
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])  # Using default starting point if none provided
    
    # Initializing and getting position map
    position_map = initialize_flexion_extension_control(starting_point)
    
    new_ie_angle = current_ie_angle + increment  # Calculating new IE angle after increment
    
    if new_ie_angle > 30:  # Checking if new angle exceeds maximum internal rotation
        print("Maximum internal rotation is 30 degrees.")
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([current_ie_angle, 1])  # Returning current angle and success flag as numpy array (limited to max)
    
    try:
        target_angle_int = int(round(current_knee_angle))  # Rounding current knee angle to nearest integer
        target_position = position_map[target_angle_int].copy()  # Getting position data for current flexion angle
        
        target_position[5] = float(new_ie_angle)  # Modifying only the yaw value for internal rotation
        
        # Printing status information
        print(f"Moving to {new_ie_angle} degrees internal rotation")
        print(f"Current flexion angle: {current_knee_angle} degrees")
        
        # Moving robot to target position
        arm.set_position(
            x=target_position[0],
            y=target_position[1],
            z=target_position[2],
            roll=target_position[3],
            pitch=target_position[4],
            yaw=target_position[5],
            speed=30,
            wait=True  # Wait until movement is complete
        )
        
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([new_ie_angle, 1])  # Returning new angle and success flag as numpy array
        
    except Exception as e:
        print(f"Error during internal rotation: {e}")
        arm.disconnect()  # Disconnecting from the robot arm on error
        return np.array([current_ie_angle, 0])  # Returning current angle and failure flag as numpy array

def external_rotation_step(increment=1, current_ie_angle=0, starting_point=None):
    """Increase external rotation by specified increment"""
    global current_knee_angle  # Using global variable for current knee flexion angle
    
    port = '192.168.1.197'  # Setting the IP address of the robot arm
    arm = XArmAPI(port)  # Creating XArmAPI object for controlling the robot arm
    arm.connect()  # Connecting to the robot arm
    arm.motion_enable(enable=True)  # Enabling motion on the robot arm
    arm.set_mode(0)  # Setting robot to position control mode
    arm.set_state(state=0)  # Setting robot to ready state
    
    if starting_point is None:
        starting_point = np.array([597, -31.4, 317.7])  # Using default starting point if none provided
    
    # Initializing and getting position map
    position_map = initialize_flexion_extension_control(starting_point)
    
    new_ie_angle = current_ie_angle - increment  # Calculating new IE angle after decrement (external is negative)
    
    if new_ie_angle < -30:  # Checking if new angle exceeds maximum external rotation
        print("Maximum external rotation is 30 degrees.")
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([current_ie_angle, 1])  # Returning current angle and success flag as numpy array (limited to max)
    
    try:
        target_angle_int = int(round(current_knee_angle))  # Rounding current knee angle to nearest integer
        target_position = position_map[target_angle_int].copy()  # Getting position data for current flexion angle
        
        target_position[5] = float(new_ie_angle)  # Modifying only the yaw value for external rotation
        
        # Printing status information
        print(f"Moving to {abs(new_ie_angle)} degrees external rotation")
        print(f"Current flexion angle: {current_knee_angle} degrees")
        
        # Moving robot to target position
        arm.set_position(
            x=target_position[0],
            y=target_position[1],
            z=target_position[2],
            roll=target_position[3],
            pitch=target_position[4],
            yaw=target_position[5],
            speed=30,
            wait=True  # Wait until movement is complete
        )
        
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([new_ie_angle, 1])  # Returning new angle and success flag as numpy array
        
    except Exception as e:
        print(f"Error during external rotation: {e}")
        arm.disconnect()  # Disconnecting from the robot arm on error
        return np.array([current_ie_angle, 0])  # Returning current angle and failure flag as numpy array

def set_internal_external_rotation(target_ie_angle, starting_point=None):
    """
    Set a specific internal/external rotation angle
    target_ie_angle: Float representing the target internal/external rotation angle in degrees (-30 to 30, positive for internal, negative for external)
    """
    port = '192.168.1.197'  # Setting the IP address of the robot arm
    arm = XArmAPI(port)  # Creating XArmAPI object for controlling the robot arm
    arm.connect()  # Connecting to the robot arm
    arm.motion_enable(enable=True)  # Enabling motion on the robot arm
    arm.set_mode(0)  # Setting robot to position control mode
    arm.set_state(state=0)  # Setting robot to ready state
    
    # Capping target angle within allowed range
    if target_ie_angle > 30:
        target_ie_angle = 30
    elif target_ie_angle < -30:
        target_ie_angle = -30
    
    try:
        # Getting current position of the robot
        current_pos = arm.get_position()
        if current_pos[0] != 0:  # Check if position was successfully retrieved
            # Extracting current position values
            x, y, z, roll, pitch, yaw = current_pos[1]
            
            # Moving robot to same position but with new yaw value for IE rotation
            arm.set_position(
                x=x, y=y, z=z,
                roll=roll, pitch=pitch, yaw=float(target_ie_angle),
                speed=30, wait=True  # Wait until movement is complete
            )
            
            arm.disconnect()  # Disconnecting from the robot arm
            return np.array([target_ie_angle, 1])  # Returning target angle and success flag as numpy array
        else:
            arm.disconnect()  # Disconnecting from the robot arm
            return np.array([0, 0])  # Returning 0 angle and failure flag as numpy array
        
    except Exception as e:
        arm.disconnect()  # Disconnecting from the robot arm on error
        return np.array([0, 0])  # Returning 0 angle and failure flag as numpy array

def varo_valgo(starting_point=None):
    """
    Function to adjust X position for varo/valgo movement
    Asks user for input directly and adjusts accordingly
    """
    port = '192.168.1.197'  # Setting the IP address of the robot arm
    arm = XArmAPI(port)  # Creating XArmAPI object for controlling the robot arm
    arm.connect()  # Connecting to the robot arm
    arm.motion_enable(enable=True)  # Enabling motion on the robot arm
    arm.set_mode(0)  # Setting robot to position control mode
    arm.set_state(state=0)  # Setting robot to ready state
    
    try:
        # Getting current position of the robot
        current_pos = arm.get_position()
        x, y, z, roll, pitch, yaw = current_pos[1]  # Extracting current position values
        
        # Asking user for adjustment amount via command line
        adjustment = float(input("Enter X adjustment in mm (positive for valgo, negative for varo): "))
        
        # Calculating new x position by adding the adjustment
        new_x = x + adjustment
        
        # Moving robot to new position with modified x value
        arm.set_position(
            x=new_x, y=y, z=z,
            roll=roll, pitch=pitch, yaw=yaw,
            speed=30, wait=True  # Wait until movement is complete
        )
        
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([new_x, 1])  # Returning new x position and success flag as numpy array
        
    except ValueError:
        # Handling invalid input (not a number)
        print("Invalid input. Please enter a number.")
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([0, 0])  # Returning 0 position and failure flag as numpy array
    except Exception as e:
        # Handling other errors
        print(f"Error: {e}")
        arm.disconnect()  # Disconnecting from the robot arm
        return np.array([0, 0])  # Returning 0 position and failure flag as numpy array

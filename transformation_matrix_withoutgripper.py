import numpy as np
import sys
import os
import time
import math
from xarm.x3 import XArm, Studio
from xarm.wrapper import XArmAPI

# Robot End Effector Points (replacing gripper points)
point_spacing = 25.4  # Distance between points in mm
starting_point = np.array([631.6240429878235, -7.514990400522947, 308.86101722717285])  # Starting point

# Create coordinate system points
p1 = starting_point + np.array([-point_spacing / 2, 0, 0])  # Left point
p2 = starting_point  # Center point
p3 = starting_point + np.array([point_spacing / 2, 0, 0])  # Right point

print(f"Robot End Points:\nPoint 1: {p1}\nCenter: {p2}\nPoint 2: {p3}")

# Robot End Effector Coordinate System
O_robot = p2  # Center point
x_robot = p3 - p1  # X-axis: vector between outer points
y_robot = np.array([0, 1, 0])  # Using global Y for simplicity
z_robot = np.cross(x_robot, y_robot)  # Z-axis: orthogonal to X and Y

# Normalize the robot axes
x_robot = x_robot / np.linalg.norm(x_robot)
y_robot = y_robot / np.linalg.norm(y_robot)
z_robot = z_robot / np.linalg.norm(z_robot)

print("\nRobot End Effector Coordinate System:")
print(f"Center (O_robot): {O_robot}")
print(f"X-axis: {x_robot}")
print(f"Y-axis: {y_robot}")
print(f"Z-axis: {z_robot}")

# Knee Coordinate System
A = np.array([620.4453110694885, 47.15324938297272, 171.99364304542542])   # Femur point A
B = np.array([626.9657611846924, -29.983650892972946, 171.94780707359314])   # Femur point B
C = np.array([626.97, 3.73791, 88.1255])   # Tibia point C
D = np.array([626.97, 3.73, 40])   # Tibia point D

Of = (A + B) / 2  # Femur center
Ot = Of + (D - C)  # Tibia center
x_knee = Of - Ot   # X-axis
y_knee = A - B     # Y-axis
z_knee = np.cross(x_knee, y_knee)  # Z-axis

# Normalize the knee axes
x_knee = x_knee / np.linalg.norm(x_knee)
y_knee = y_knee / np.linalg.norm(y_knee)
z_knee = z_knee / np.linalg.norm(z_knee)

print("\nKnee Coordinate System:")
print(f"Femur Center (Of): {Of}")
print(f"Tibia Center (Ot): {Ot}")
print(f"X-axis: {x_knee}")
print(f"Y-axis: {y_knee}")
print(f"Z-axis: {z_knee}")

# Compute Transformation Matrix
R_knee_to_robot = np.array([x_robot, y_robot, z_robot]).T @ np.array([x_knee, y_knee, z_knee])
translation = O_robot - Of  # Robot center to femur center
transformation_matrix = np.eye(4)
transformation_matrix[:3, :3] = R_knee_to_robot
transformation_matrix[:3, 3] = translation

print("\nTransformation Matrix (Knee Relative to Robot):")
print(transformation_matrix)

def calculate_gs_angles(transformation_matrix):
    """
    Calculate Grood-Suntay angles from transformation matrix
    Returns angles in degrees [IE, FE, VV]
    """
    # we grab just the rotation part of the matrix (top left 3x3)
    Tft = transformation_matrix[:3, :3]
    
    # these are the axes for the femur, they don't move
    Fx = np.array([1, 0, 0])
    Fy = np.array([0, 1, 0])
    
    # grab the new positions of the tibia axes after the movement
    Tft_x = Tft[:, 0]  # x-axis of tibia 
    Tft_y = Tft[:, 1]  # y-axis of tibia
    
    #  make a "floating axis" that both bones share
    #  crossing the axes
    e2 = np.cross(Tft_x, Fy)
    e2_norm = np.linalg.norm(e2)  # make sure it's length 1
    e2_unit = e2 / e2_norm
    
    # we can get the three angles that describe the knee movement:
    
    # 1. internal/external rotation 
    #  angles could wrap around
    output = np.cross(e2_unit, Fx)
    if output[1] > 0:
        alpha = np.arcsin(np.dot(e2_unit, Fx)) * 180/np.pi
    else:
        alpha = -180 - np.arcsin(np.dot(e2_unit, Fx)) * 180/np.pi
    
    # flexion/extension 
    # 90 minus arccos gives us the angle from the straight leg position
    beta = 90 - np.arccos(np.dot(Fy, Tft_x)) * 180/np.pi
    
    # varus/valgus 
    # this is the angle between the floating axis and tibia y-axis
    gamma = np.arcsin(np.dot(e2_unit, Tft_y)) * 180/np.pi
    
    return np.array([gamma, alpha, beta])

def create_gs_rotation_matrix(ie_angle, fe_angle, vv_angle):
    """
    Create rotation matrix from Grood-Suntay angles
    Angles should be in degrees
    """
    # convert our angles to radians 
    ie = np.radians(ie_angle)
    fe = np.radians(fe_angle)
    vv = np.radians(vv_angle)
    
    # make three rotation matrices, one for each type of movement
    # each one rotates around a different axis
    
    # knee bending (flexion/extension)
    R_fe = np.array([
        [np.cos(fe), -np.sin(fe), 0],
        [np.sin(fe), np.cos(fe), 0],
        [0, 0, 1]
    ])
    
    #  (internal/external) twisting
    R_ie = np.array([
        [1, 0, 0],
        [0, np.cos(ie), -np.sin(ie)],
        [0, np.sin(ie), np.cos(ie)]
    ])
    
    #  knee knocking (varus/valgus)
    R_vv = np.array([
        [np.cos(vv), 0, -np.sin(vv)],
        [0, 1, 0],
        [np.sin(vv), 0, np.cos(vv)]
    ])
    
    # combine all three rotations - the order does matter 
    # we do varus/valgus, then flexion/extension, then internal/external
    
    return R_vv @ R_fe @ R_ie
    
def test_gs_rotations(fe_angle, ie_angle=0, vv_angle=0):
    """
    Test rotations using Grood-Suntay angles
    fe_angle: Flexion/Extension angle in degrees
    ie_angle: Internal/External rotation angle in degrees
    vv_angle: Varus/Valgus angle in degrees
    """
    # Create rotation matrix using GS angles
    rotation = create_gs_rotation_matrix(ie_angle, fe_angle, vv_angle)
    
    # Convert points to homogeneous coordinates
    Of_homog = np.append(Of, 1)
    pivot_knee = np.linalg.inv(transformation_matrix) @ Of_homog
    
    head_homog = np.append(starting_point, 1)
    head_knee = np.linalg.inv(transformation_matrix) @ head_homog
    
    # Apply rotation in knee coordinate system
    pivot_to_head = head_knee[:3] - pivot_knee[:3]
    rotated_vector = rotation @ pivot_to_head
    new_head_knee = pivot_knee[:3] + rotated_vector
    
    # Transform back to global coordinates
    new_head_homog = np.append(new_head_knee, 1)
    new_head_global = transformation_matrix @ new_head_homog
    
    # Calculate and print GS angles of the transformation
    print(f"\nRequested angles:")
    print(f"Flexion/Extension: {fe_angle}°")
    print(f"Internal/External Rotation: {ie_angle}°")
    print(f"Varus/Valgus: {vv_angle}°")
    
    print(f"\nPivot point (knee): {Of}")
    print(f"Original position: {starting_point}")
    print(f"New position: {new_head_global[:3]}")
    
    return new_head_global[:3]

def test_different_movements(arm, test_angles):
    try:
        start_pos = test_gs_rotations(0)
        
        print("\n1. Testing Flexion/Extension")
        for angle in test_angles:
            pos = test_gs_rotations(fe_angle=angle)
            print(f"\nMoving to {angle} degrees flexion")
            arm.set_position(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]), 
                           wait=True, speed=30)
            time.sleep(2)
        
        print("\n2. Testing Combined Movement")
        for fe_angle in test_angles:
            # Add small IE and VV rotations
            pos = test_gs_rotations(fe_angle=fe_angle, 
                                  ie_angle=5, 
                                  vv_angle=2)
            print(f"\nMoving to position with combined rotation")
            arm.set_position(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]), 
                           wait=True, speed=30)
            time.sleep(2)
        
        print("\nReturning to start position")
        arm.set_position(x=float(start_pos[0]), y=float(start_pos[1]), 
                        z=float(start_pos[2]), wait=True)
        
    except Exception as e:
        print(f"Error occurred: {e}")
        arm.emergency_stop()

#Test angles
port = '192.168.1.197'
arm = XArmAPI(port)
arm.connect()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
#arm.set_gripper_position(500)
arm.move_gohome()
#arm.set_gripper_enable(1)
#arm.set_gripper_position(300)
# Move to starting position
print("\nMoving to starting position...")
arm.set_position(
    x=starting_point[0],
    y=starting_point[1],
    z=starting_point[2]+20,  # Added 20mm for safety
    roll=-180,
    pitch=0,
    yaw=0,
    speed=50,
    is_radian=False,
    wait=True
)
time.sleep(2)  

try:
    # Test pure flexion movements
    test_angles = [0, 30, 45, 60]
    
    print("\nTesting flexion movements...")
    for angle in test_angles:
        pos_flexion = test_gs_rotations(fe_angle=angle)
        print(f"\nMoving to {angle} degrees flexion")
        print(f"Target position: {pos_flexion}")
        
        arm.set_position(
            x=float(pos_flexion[0]),
            y=float(pos_flexion[1]),
            z=float(pos_flexion[2]),
            roll=-180,
            pitch=0,
            yaw=0,
            speed=30,
            wait=True
        )
        time.sleep(2)  # Wait between movements
    
    # Return to starting position
    print("\nReturning to starting position...")
    arm.set_position(
        x=starting_point[0],
        y=starting_point[1],
        z=starting_point[2]+20,  # Adding safety offset
        roll=-180,
        pitch=0,
        yaw=0,
        speed=50,
        wait=True
    )

except Exception as e:
    print(f"Error occurred: {e}")
    arm.emergency_stop()

finally:
    arm.disconnect()

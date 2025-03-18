import numpy as np
import sys
import os
import time
import math
from xarm.x3 import XArm, Studio
from xarm.wrapper import XArmAPI

# Robot End Effector Points (replacing gripper points)
point_spacing = 25.4  # Distance between points in mm
starting_point = np.array([597, -31.4, 317.7])  # Starting point

# Create coordinate system points
p1 = starting_point + np.array([-point_spacing / 2, 0, 0])  # Left point
p2 = starting_point  # Center point
p3 = starting_point + np.array([point_spacing / 2, 0, 0])  # Right point

print(f"Robot End Points:\nPoint 1: {p1}\nCenter: {p2}\nPoint 2: {p3}")

# Robot End Effector Coordinate System
O_robot = p2  # Center point
x_robot = p3 - p1  # x axis vector between outer points
y_robot = np.array([0, 1, 0])  # Using global Y for simplicity
z_robot = np.cross(x_robot, y_robot)  # z axis, making sure it is orthogonal to X and Y

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

'''# Knee Coordinate System
A = np.array([580.75, -225.58, 172.04])   # Femur point A
B = np.array([706.26, -225.65, 171.65])   # Femur point B
C = np.array([582.72, 41.862, 168.65])   # Tibia point C
D = np.array([587.38, 438.42, 172.19])   # Tibia point D'''

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
    Calculating the Grood-Suntay angles from transformation matrix
    Returns angles in degrees [IE, FE, VV]
    """
    # Extracting the rotation matrix
    Tft = transformation_matrix[:3, :3]
    
    # Define femur axes (this is a fixed coordinate system)
    Fx = np.array([1, 0, 0])
    Fy = np.array([0, 1, 0])
    
    # Get tibia axes from transformation
    Tft_x = Tft[:, 0]  #  x-axis
    Tft_y = Tft[:, 1]  #  y-axis
    
    # Calculate floating axis (e2)
    e2 = np.cross(Tft_x, Fy)
    e2_norm = np.linalg.norm(e2)
    e2_unit = e2 / e2_norm
    
    # Calculating Grood-Suntay angles
    # Internal/External rotation (around fixed axis)
    output = np.cross(e2_unit, Fx)
    if output[1] > 0:
        alpha = np.arcsin(np.dot(e2_unit, Fx)) * 180/np.pi
    else:
        alpha = -180 - np.arcsin(np.dot(e2_unit, Fx)) * 180/np.pi
    
    # Flexion/Extension 
    beta = 90 - np.arccos(np.dot(Fy, Tft_x)) * 180/np.pi
    
    # Varus/Valgus 
    gamma = np.arcsin(np.dot(e2_unit, Tft_y)) * 180/np.pi
    
    return np.array([gamma, alpha, beta])

def create_gs_rotation_matrix(ie_angle, fe_angle, vv_angle):
    """
    Create rotation matrix from Grood-Suntay angles
    Angles should be in degrees
    """
    # Convert to radians
    ie = np.radians(ie_angle)
    fe = np.radians(fe_angle)
    vv = np.radians(vv_angle)
    
    # Create individual rotation matrices
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
    
    # Combine rotations (order matters!)
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

def flexion_extension_control(arm, transformation_matrix, Of, starting_point):
    """
    Combined function for flexion and extension control using pre-calculated paths.
    - Positive angles = flexion
    - Negative angles = extension (relative to current flexion angle)
    I am using a plan calculated all positions corresponding to each angle beforehand and going to the position once given give the angle input
    
    Parameters:
    arm - The robot arm object
    transformation_matrix - The transformation matrix between knee and robot coordinates
    Of - The femur center point
    starting_point - The robot starting point
    
    Returns:
    The final knee angle
    """
    global current_knee_angle
    
    try:
        # Reset global angle to start fresh
        current_knee_angle = 0.0
        
        # Pre-calculate paths for all angles from 0 to 120
        angle_range = list(range(0, 121))  # 0, 1, 2, ..., 120 degrees
        position_map = {}
        
        print("Pre-calculating positions for all angles...")
        for angle in angle_range:
            # Calculate position using GS rotation
            pos = test_gs_rotations(fe_angle=angle)
            # Calculate roll value
            roll_val = -180 + angle
            # Store full position (x, y, z, roll, pitch, yaw)
            position_map[angle] = [
                float(pos[0]), float(pos[1]), float(pos[2]), 
                roll_val, 0, 0
            ]
        
        print("\n===== Flexion/Extension Control =====")
        print("Enter positive number for flexion, negative for extension")
        print("Extension is relative to current flexion (example is that -20 means extend 20 degrees from current position)")
        print("Enter 'q' to quit")
        print(f"Current angle: {current_knee_angle:.1f} degrees")
        
        while True:
            user_input = input("\nEnter angle value or 'q' to quit: ")
            
            if user_input.lower() == 'q':
                print("Exiting flexion/extension control.")
                break
            
            try:
                input_value = float(user_input)
                
                # Determine if this is flexion or extension
                is_flexion = input_value >= 0
                
                if is_flexion:
                    # FLEXION - direct angle input
                    target_angle = input_value
                    
                    # Apply angle limits
                    if target_angle > 120:
                        print("Maximum flexion angle is 120 degrees. Setting to 120.")
                        target_angle = 120
                    
                    # Get the pre-calculated position
                    target_angle_int = int(round(target_angle))
                    target_position = position_map[target_angle_int]
                    
                    print(f"\nMoving to {target_angle_int} degrees flexion")
                    print(f"Using roll value: {target_position[3]}")
                    
                    # Move to the position
                    arm.set_position(
                        x=target_position[0],
                        y=target_position[1],
                        z=target_position[2],
                        roll=target_position[3],
                        pitch=target_position[4],
                        yaw=target_position[5],
                        speed=30,
                        wait=True
                    )
                    
                    # Update current angle
                    current_knee_angle = target_angle_int
                    
                else:
                    # EXTENSION - relative to current angle
                    extension_amount = abs(input_value)
                    
                    # Check if trying to extend more than the current flexion
                    if extension_amount > current_knee_angle:
                        print(f"Warning: Cannot extend {extension_amount} degrees from current position.")
                        print(f"Maximum extension from current position is {current_knee_angle} degrees.")
                        continue
                    
                    # Calculate new angle after extension
                    target_angle = current_knee_angle - extension_amount
                    target_angle_int = int(round(target_angle))
                    
                    # Get the pre-calculated position
                    target_position = position_map[target_angle_int]
                    
                    print(f"\nExtending by {extension_amount} degrees")
                    print(f"Moving to {target_angle_int} degrees flexion")
                    print(f"Using roll value: {target_position[3]}")
                    
                    # Move to the position
                    arm.set_position(
                        x=target_position[0],
                        y=target_position[1],
                        z=target_position[2],
                        roll=target_position[3],
                        pitch=target_position[4],
                        yaw=target_position[5],
                        speed=30,
                        wait=True
                    )
                    
                    # Update current angle
                    current_knee_angle = target_angle_int
                
                time.sleep(1)  # Wait between movements
                
            except ValueError:
                print("Invalid input. Please enter a number.")
        
        # Return to starting position
        print("\nReturning to starting position...")
        arm.set_position(
            x=starting_point[0],
            y=starting_point[1],
            z=starting_point[2],  # Adding safety offset
            roll=-180,
            pitch=0,
            yaw=0,
            speed=50,
            wait=True
        )
        
        # Reset current angle
        current_knee_angle = 0.0
        
    except Exception as e:
        print(f"Error occurred: {e}")
        try:
            arm.emergency_stop()
        except:
            pass
    
    return current_knee_angle

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
    z=starting_point[2],  # Added 20mm for safety
    roll=-180,
    pitch=0,
    yaw=0,
    speed=80,
    is_radian=False,
    wait=True
)
time.sleep(2)  

try:
    current_knee_angle = 0.0
    while True:
        print("\n===== Knee Movement Control Test =====")
        print("1. Test Flexion/Extension Control")
        print("2. Exit")
        
        choice = input("Enter your choice (1-2): ")
        
        if choice == '1':
            # Test combined flexion/extension control
            flexion_extension_control(arm, transformation_matrix, Of, starting_point)
        elif choice == '2':
            print("Exiting test menu.")
            break
        else:
            print("Invalid choice. Please enter 1-2.")
    
    # Return to starting position
    print("\nReturning to starting position...")
    arm.set_position(
        x=starting_point[0],
        y=starting_point[1],
        z=starting_point[2],  # Adding safety offset
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

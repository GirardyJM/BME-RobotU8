import numpy as np
import sys
import os
import time
import math
from xarm.x3 import XArm, Studio
from xarm.wrapper import XArmAPI


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

def initialize_flexion_extension(arm, transformation_matrix, Of, starting_point):
    """
    Initialize the flexion/extension control system by pre-calculating positions.
    Must be called once before using flexion_step or extension_step.
    
    Parameters:
    arm - The robot arm object
    transformation_matrix - The transformation matrix between knee and robot coordinates
    Of - The femur center point
    starting_point - The robot starting point
    
    Returns:
    position_map - Dictionary containing pre-calculated positions for all angles
    """
    global current_knee_angle
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
    
    # Reset arm to starting position to ensure we begin at 0 degrees
    arm.set_position(
        x=starting_point[0],
        y=starting_point[1],
        z=starting_point[2],
        roll=-180,
        pitch=0,
        yaw=0,
        speed=50,
        wait=True
    )
    
    return position_map

def flexion_step(arm, position_map, increment=1, log_file="flexion_force_log2.csv"):
    """
    Move the robot arm to increase flexion by the specified increment.
    
    Parameters:
    arm - The robot arm object
    position_map - Dictionary of pre-calculated positions (from initialize_flexion_extension)
    increment - Amount to increase flexion angle (default: 1 degree)
    
    Returns:
    current_angle - The new knee angle after movement
    success - Boolean indicating if movement was successful
    """
    global current_knee_angle
    
    # Calculate new angle
    new_angle = current_knee_angle + increment
    
    # Apply angle limits
    if new_angle > 120:
        print("Maximum flexion angle is 120 degrees.")
        return current_knee_angle, False
    
    # Get the pre-calculated position
    target_angle_int = int(round(new_angle))
    target_position = position_map[target_angle_int]
    
    print(f"Moving to {target_angle_int} degrees flexion")
    print(f"Using roll value: {target_position[3]}")
    
    try:
        arm.ft_sensor_enable(1)         # Enable force sensor
        time.sleep(0.3)                 # Small delay to ensure sensor readiness

        # Get current arm position
        code, current_pos = arm.get_position()
        if code != 0:
            raise Exception("Could not get current position")

        current_xyz = current_pos[:3]
        target_xyz = target_position[:3]

        # Measure distance to determine if movement is needed
        distance = math.sqrt(sum([(target_xyz[i] - current_xyz[i]) ** 2 for i in range(3)]))
        print(f"Distance to move: {distance:.4f} mm")

        if distance < 0.5:
            print("Distance too small, forcing Z offset to trigger movement")
            target_position[2] += 1.0  # Add 1mm Z offset to force movement

        # Move to the position
        arm.set_position(
            x=target_position[0],
            y=target_position[1],
            z=target_position[2],
            roll=target_position[3],
            pitch=target_position[4],
            yaw=target_position[5],
            speed=30,
            wait=False
        )
        time.sleep(0.5)  # Allow system to stabilize before reading force
        force = arm.ft_ext_force
        timestamp = time.time()

        # Log data
        with open(log_file, 'a') as f:
            f.write(f"{timestamp},{target_angle_int},{force[0]},{force[1]},{force[2]},{force[3]},{force[4]},{force[5]}\n")

        arm.ft_sensor_enable(0)  # Disable after reading

        # Update current angle
        current_knee_angle = target_angle_int
        return current_knee_angle, True
        
    except Exception as e:
        print(f"Error during flexion: {e}")
        return current_knee_angle, False

def extension_step(arm, position_map, increment=1):
    """
    Move the robot arm to increase extension by the specified increment.
    
    Parameters:
    arm - The robot arm object
    position_map - Dictionary of pre-calculated positions (from initialize_flexion_extension)
    increment - Amount to increase extension angle (default: 1 degree)
    
    Returns:
    current_angle - The new knee angle after movement
    success - Boolean indicating if movement was successful
    """
    global current_knee_angle
    
    # Calculate new angle
    new_angle = current_knee_angle - increment
    
    # Apply angle limits
    if new_angle < 0:
        print("Cannot extend beyond 0 degrees.")
        return current_knee_angle, False
    
    # Get the pre-calculated position
    target_angle_int = int(round(new_angle))
    target_position = position_map[target_angle_int]
    
    print(f"Moving to {target_angle_int} degrees flexion (extending)")
    print(f"Using roll value: {target_position[3]}")
    
    try:
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
        return current_knee_angle, True
        
    except Exception as e:
        print(f"Error during extension: {e}")
        return current_knee_angle, False

def set_flexion_angle(arm, position_map, target_angle, log_file="smooth_flexion_force_log.csv"):
    """
    Move the robot directly to the target angle. While moving, poll current position
    and log continuous force readings along the way.
    """
    global current_knee_angle

    if target_angle < 0 or target_angle > 120:
        print("Target angle out of range (0–120).")
        return current_knee_angle, False

    target_angle_int = int(round(target_angle))
    target_position = position_map[target_angle_int]
    print(f"Moving to {target_angle_int}° flexion...")

    try:

        # Start non-blocking move
        arm.set_position(
            x=target_position[0],
            y=target_position[1],
            z=target_position[2],
            roll=target_position[3],
            pitch=target_position[4],
            yaw=target_position[5],
            speed=30,
            wait=False
        )

        last_pos = None
        still_counter = 0
        max_still_count = 6  # ~300 ms stillness means motion is done

        while arm.connected and arm.error_code == 0:
            code, pos = arm.get_position()
            if code != 0:
                break

            current_xyz = pos[:3]
            timestamp = time.time()
            force = arm.ft_ext_force

            # Find the closest angle in the position_map
            closest_angle = min(
                position_map.keys(),
                key=lambda angle: np.linalg.norm(np.array(position_map[angle][:3]) - np.array(current_xyz))
            )

            # Log force with timestamp and closest angle
            with open(log_file, 'a') as f:
                f.write(f"{timestamp},{closest_angle},{current_xyz[0]},{current_xyz[1]},{current_xyz[2]},"
                        f"{force[0]},{force[1]},{force[2]},{force[3]},{force[4]},{force[5]}\n")

            # Detect motion stop
            if last_pos:
                delta = math.sqrt(sum((current_xyz[i] - last_pos[i]) ** 2 for i in range(3)))
                if delta < 0.01:
                    still_counter += 1
                else:
                    still_counter = 0
            last_pos = current_xyz

            if still_counter >= max_still_count:
                print("Movement complete.")
                break

            time.sleep(0.05)

        current_knee_angle = target_angle_int
        return current_knee_angle, True

    except Exception as e:
        print(f"Error during set_flexion_angle: {e}")
        return current_knee_angle, False





def reset_to_starting_position(arm, starting_point):
    """
    Return the robot arm to the starting position.
    
    Parameters:
    arm - The robot arm object
    starting_point - The starting position coordinates
    
    Returns:
    success - Boolean indicating if movement was successful
    """
    global current_knee_angle
    
    try:
        # Return to starting position
        print("Returning to starting position...")
        arm.set_position(
            x=starting_point[0],
            y=starting_point[1],
            z=starting_point[2],
            roll=-180,
            pitch=0,
            yaw=0,
            speed=50,
            wait=True
        )
        
        # Reset current angle
        current_knee_angle = 0.0
        return True
        
    except Exception as e:
        print(f"Error returning to starting position: {e}")
        return False

def get_current_knee_angle():
    """
    Get the current knee angle.
    
    Returns:
    current_angle - The current knee angle
    """
    global current_knee_angle
    return current_knee_angle

def test_labview_compatible_functions():
    """
    Test script for the LabVIEW-compatible flexion/extension functions.
    """
    global current_knee_angle
    
    # Setup XArm
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    arm.move_gohome()
    
    # Move to starting position
    print("\nMoving to starting position...")
    arm.set_position(
        x=starting_point[0],
        y=starting_point[1],
        z=starting_point[2],
        roll=-180,
        pitch=0,
        yaw=0,
        speed=80,
        is_radian=False,
        wait=True
    )
    time.sleep(2)
    
    try:
        # Step 1: Initialize the system
        print("\nStep 1: Initializing flexion/extension system...")
        position_map = initialize_flexion_extension(arm, transformation_matrix, Of, starting_point)
        print(f"Current knee angle: {get_current_knee_angle()}")
        
        # Step 2: Test flexion steps
        print("\nStep 2: Testing flexion steps...")
        print("Performing 5 flexion steps (1 degree each)")
        for i in range(5):
            new_angle, success = flexion_step(arm, position_map)
            print(f"Flexion step {i+1}: new angle = {new_angle}, success = {success}")
            time.sleep(1)
        
        # Step 3: Test direct angle setting
        print("\nStep 3: Testing direct angle setting...")
        test_angle = 30
        print(f"Setting angle directly to {test_angle} degrees")
        new_angle, success = set_flexion_angle(arm, position_map, test_angle)
        print(f"Direct angle set: new angle = {new_angle}, success = {success}")
        time.sleep(2)
        
        # Step 4: Test extension steps
        print("\nStep 4: Testing extension steps...")
        print("Performing 5 extension steps (1 degree each)")
        for i in range(5):
            new_angle, success = extension_step(arm, position_map)
            print(f"Extension step {i+1}: new angle = {new_angle}, success = {success}")
            time.sleep(1)
        
        # Step 5: Return to starting position
        print("\nStep 5: Returning to starting position...")
        success = reset_to_starting_position(arm, starting_point)
        print(f"Reset to starting position: success = {success}")
        print(f"Final knee angle: {get_current_knee_angle()}")
        
    except Exception as e:
        print(f"Test error: {e}")
        try:
            arm.emergency_stop()
        except:
            pass
    
    finally:
        # Disconnect arm
        arm.disconnect()
        print("Test complete, arm disconnected.")
def internal_rotation_step(arm, position_map, increment=1, current_ie_angle=0):
    """
    Increase internal rotation by specified increment (positive yaw)
    
    Parameters:
    arm - The robot arm object
    position_map - Dictionary of pre-calculated positions
    increment - Amount to increase internal rotation (default: 1 degree)
    current_ie_angle - Current internal/external rotation angle
    
    Returns:
    new_ie_angle - The new internal/external rotation angle
    success - Boolean indicating if movement was successful
    """
    global current_knee_angle
    
    # Calculate new IE angle
    new_ie_angle = current_ie_angle + increment
    
    # Apply angle limits (typically ±30 degrees for IE rotation)
    if new_ie_angle > 30:
        print("Maximum internal rotation is 30 degrees.")
        return current_ie_angle, False
    
    try:
        # Get the position for the current flexion angle
        target_angle_int = int(round(current_knee_angle))
        target_position = position_map[target_angle_int].copy()
        
        # Modify the yaw value for internal rotation
        target_position[5] = float(new_ie_angle)  # yaw value is at index 5
        
        # Move the robot arm
        print(f"Moving to {new_ie_angle} degrees internal rotation")
        print(f"Current flexion angle: {current_knee_angle} degrees")
        
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
        
        return new_ie_angle, True
        
    except Exception as e:
        print(f"Error during internal rotation: {e}")
        return current_ie_angle, False

def external_rotation_step(arm, position_map, increment=1, current_ie_angle=0):
    """
    Increase external rotation by specified increment (negative yaw)
    
    Parameters:
    arm - The robot arm object
    position_map - Dictionary of pre-calculated positions
    increment - Amount to increase external rotation (default: 1 degree)
    current_ie_angle - Current internal/external rotation angle
    
    Returns:
    new_ie_angle - The new internal/external rotation angle
    success - Boolean indicating if movement was successful
    """
    global current_knee_angle
    
    # Calculate new IE angle (more negative for external rotation)
    new_ie_angle = current_ie_angle - increment
    
    # Apply angle limits (typically ±30 degrees for IE rotation)
    if new_ie_angle < -30:
        print("Maximum external rotation is 30 degrees.")
        return current_ie_angle, False
    
    try:
        # Get the position for the current flexion angle
        target_angle_int = int(round(current_knee_angle))
        target_position = position_map[target_angle_int].copy()
        
        # Modify the yaw value for external rotation
        target_position[5] = float(new_ie_angle)  # yaw value is at index 5
        
        # Move the robot arm
        print(f"Moving to {abs(new_ie_angle)} degrees external rotation")
        print(f"Current flexion angle: {current_knee_angle} degrees")
        
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
        
        return new_ie_angle, True
        
    except Exception as e:
        print(f"Error during external rotation: {e}")
        return current_ie_angle, False

def set_internal_external_rotation(arm, position_map, target_ie_angle):
    """
    Set a specific internal/external rotation angle
    
    Parameters:
    arm - The robot arm object
    position_map - Dictionary of pre-calculated positions
    target_ie_angle - The desired IE angle (positive for internal, negative for external)
    
    Returns:
    target_ie_angle - The set internal/external rotation angle
    success - Boolean indicating if movement was successful
    """
    global current_knee_angle
    
    # Apply angle limits (typically ±30 degrees for IE rotation)
    if target_ie_angle > 30:
        print("Maximum internal rotation is 30 degrees. Setting to 30.")
        target_ie_angle = 30
    elif target_ie_angle < -30:
        print("Maximum external rotation is 30 degrees. Setting to -30.")
        target_ie_angle = -30
    
    try:
        # Get the position for the current flexion angle
        target_angle_int = int(round(current_knee_angle))
        target_position = position_map[target_angle_int].copy()
        
        # Modify the yaw value for rotation
        target_position[5] = float(target_ie_angle)  # yaw value is at index 5
        
        # Determine rotation type for display
        rotation_type = "internal" if target_ie_angle >= 0 else "external"
        display_angle = abs(target_ie_angle)
        
        # Move the robot arm
        print(f"Moving to {display_angle} degrees {rotation_type} rotation")
        print(f"Current flexion angle: {current_knee_angle} degrees")
        
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
        
        return target_ie_angle, True
        
    except Exception as e:
        print(f"Error setting internal/external rotation: {e}")
        return 0, False
# To run the test, add this to your main section:
# current_knee_angle = 0.0  # Initialize global variable if not already defined
# test_labview_compatible_functions()
def varo_valgo():
    '''
    Function to adjust X position for varo/valgo movement
    Asks user for input directly and adjusts accordingly
    '''
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    
    try:
        # Get current position
        current_pos = arm.get_position()
        x, y, z, roll, pitch, yaw = current_pos[1]
        
        # Ask user for adjustment amount
        adjustment = float(input("Enter X adjustment in mm (positive for valgo, negative for varo): "))
        
        # Calculate new x position
        new_x = x + adjustment
        
        # Move to new position
        arm.set_position(
            x=new_x, y=y, z=z,
            roll=roll, pitch=pitch, yaw=yaw,
            speed=30, wait=True
        )
        
        arm.disconnect()
        return np.array([new_x, 1])
        
    except ValueError:
        print("Invalid input. Please enter a number.")
        arm.disconnect()
        return np.array([0, 0])
    except Exception as e:
        print(f"Error: {e}")
        arm.disconnect()
        return np.array([0, 0])

if __name__ == "__main__":
    # Initialize global variable

    current_knee_angle = 0.0
    point_spacing = 25.4  # Distance between points in mm
    starting_point = np.array([597, -31.4, 350.7])  # Starting point

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
    
    # Setup XArm
    port = '192.168.1.197'
    arm = XArmAPI(port)
    arm.motion_enable(enable=True)
    arm.clean_error()
    arm.clean_warn()
    arm.ft_sensor_enable(1)  
    time.sleep(0.5)
    arm.ft_sensor_set_zero()
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    arm.move_gohome()
    
    
    # Move to starting position
    print("\nMoving to starting position...")
    arm.set_position(
        x=starting_point[0],
        y=starting_point[1],
        z=starting_point[2],
        roll=-180,
        pitch=0,
        yaw=0,
        speed=80,
        is_radian=False,
        wait=True
    )

    is_moving = True
    time.sleep(2)
    
    try:
    # Initialize the system
        print("Initializing flexion/extension system...")
        position_map = initialize_flexion_extension(arm, transformation_matrix, Of, starting_point)
        
        # Track internal/external rotation
        current_ie_angle = 0.0
        
        while is_moving and arm.connected and arm.error_code == 0:
            print("\n===== Test Menu =====")
            print("1. Flexion step (1 degree)")
            print("2. Extension step (1 degree)")
            print("3. Internal rotation step (1 degree)")
            print("4. External rotation step (1 degree)")
            print("5. Set specific flexion angle")
            print("6. Set specific internal/external rotation")
            print("7. Return to starting position")
            print("8. Varo-Valgo")
            print("Exit")
            
            choice = input("Enter your choice (1-8): ")
            
            if choice == '1':
                new_angle, success = flexion_step(arm, position_map, log_file="flexion_force_log2.csv")
                print(f"Flexion step: new angle = {new_angle}, success = {success}")
            
            elif choice == '2':
                new_angle, success = extension_step(arm, position_map)
                print(f"Extension step: new angle = {new_angle}, success = {success}")
            
            elif choice == '3':
                new_ie_angle, success = internal_rotation_step(arm, position_map, 1, current_ie_angle)
                if success:
                    current_ie_angle = new_ie_angle
                print(f"Internal rotation step: new IE angle = {current_ie_angle}, success = {success}")
            
            elif choice == '4':
                new_ie_angle, success = external_rotation_step(arm, position_map, 1, current_ie_angle)
                if success:
                    current_ie_angle = new_ie_angle
                print(f"External rotation step: new IE angle = {current_ie_angle}, success = {success}")
            
            elif choice == '5':
                try:
                    angle = float(input("Enter desired flexion angle (0-120): "))
                    new_angle, success = set_flexion_angle(arm, position_map, angle, log_file="smooth_flexion_force_log.csv")
                    print(f"Set flexion angle: new angle = {new_angle}, success = {success}")
                except ValueError:
                    print("Invalid input. Please enter a number.")
            
            elif choice == '6':
                try:
                    angle = float(input("Enter desired IE angle (-30 to 30, negative for external): "))
                    new_ie_angle, success = set_internal_external_rotation(arm, position_map, angle)
                    if success:
                        current_ie_angle = new_ie_angle
                    print(f"Set IE angle: new angle = {current_ie_angle}, success = {success}")
                except ValueError:
                    print("Invalid input. Please enter a number.")
            
            elif choice == '7':
                success = reset_to_starting_position(arm, starting_point)
                current_ie_angle = 0.0  # Reset IE angle when returning to start
                print(f"Reset to starting position: success = {success}")

            elif choice == '8':
                    result = varo_valgo()
                    if result[1] == 1:
                        print(f"Successfully moved to X position: {result[0]}")
                    else:
                        print("Movement failed")
            
            elif choice == '9':
                print("Exiting test menu.")
                break
            
            else:
                print("Invalid choice. Please enter 1-8.")
    
    except Exception as e:
        print(f"Error: {e}")
        try:
            arm.emergency_stop()
        except:
            pass
    
    finally:
        # Reset and disconnect
        try:
            reset_to_starting_position(arm, starting_point)
        except:
            pass
        arm.disconnect()
        print("Test complete, arm disconnected.")
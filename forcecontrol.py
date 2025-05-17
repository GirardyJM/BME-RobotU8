import numpy as np
import sys
import os
import time
import math
import csv
from xarm.wrapper import XArmAPI

# Global variable to track the current knee angle across functions
current_knee_angle = 0.0

# Force safety thresholds - adjust based on your requirements
MAX_SAFE_FORCE = 50.0  # N
MAX_SAFE_TORQUE = 2.0  # Nm

# Damping and stiffness parameters (set to 0 initially, update with actual values)
JOINT_STIFFNESS = 0.0  # Nm/deg - TODO: input actual stiffness value
JOINT_DAMPING = 0.0    # Nm·s/deg - TODO: input actual damping value
JOINT_NEUTRAL_POSITION = np.zeros(6)  # Reference pose for stiffness calculation - TODO: set based on desired neutral joint position

def calculate_gs_angles(transformation_matrix):
    Tft = transformation_matrix[:3, :3]
    Fx = np.array([1, 0, 0])
    Fy = np.array([0, 1, 0])
    Tft_x = Tft[:, 0]
    Tft_y = Tft[:, 1]
    e2 = np.cross(Tft_x, Fy)
    e2_unit = e2 / np.linalg.norm(e2)
    output = np.cross(e2_unit, Fx)
    if output[1] > 0:
        alpha = np.arcsin(np.dot(e2_unit, Fx)) * 180/np.pi
    else:
        alpha = -180 - np.arcsin(np.dot(e2_unit, Fx)) * 180/np.pi
    beta = 90 - np.arccos(np.dot(Fy, Tft_x)) * 180/np.pi
    gamma = np.arcsin(np.dot(e2_unit, Tft_y)) * 180/np.pi
    return np.array([gamma, alpha, beta])

def create_gs_rotation_matrix(ie_angle, fe_angle, vv_angle):
    ie, fe, vv = np.radians([ie_angle, fe_angle, vv_angle])
    R_fe = np.array([[np.cos(fe), -np.sin(fe), 0], [np.sin(fe), np.cos(fe), 0], [0, 0, 1]])
    R_ie = np.array([[1, 0, 0], [0, np.cos(ie), -np.sin(ie)], [0, np.sin(ie), np.cos(ie)]])
    R_vv = np.array([[np.cos(vv), 0, -np.sin(vv)], [0, 1, 0], [np.sin(vv), 0, np.cos(vv)]])
    return R_vv @ R_fe @ R_ie

def test_gs_rotations(fe_angle, Of, starting_point, transformation_matrix, ie_angle=0, vv_angle=0):
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
    angle_range = list(range(0, 121))
    position_map = {}
    print("Pre-calculating positions for all angles...")
    for angle in angle_range:
        pos = test_gs_rotations(fe_angle=angle, Of=Of, starting_point=starting_point, transformation_matrix=transformation_matrix)
        roll_val = -180 + angle
        position_map[angle] = [
            float(pos[0]), float(pos[1]), float(pos[2]),
            roll_val, 0, 0
        ]
    arm.set_position(
        x=starting_point[0], y=starting_point[1], z=starting_point[2],
        roll=-180, pitch=0, yaw=0, speed=50, wait=True
    )
    return position_map

def transform_wrench(wrench, r_offset):
    F = np.array(wrench[:3])
    T = np.array(wrench[3:])
    r = np.array(r_offset)
    T_new = T + np.cross(r, F)
    return np.concatenate((F, T_new))

def is_force_safe(force_vector):
    return all(abs(f) <= MAX_SAFE_FORCE for f in force_vector[:3]) and all(abs(t) <= MAX_SAFE_TORQUE for t in force_vector[3:])

def measure_passive_forces(arm, position_map, r_offset, output_csv="passive_forces.csv"):
    passive_forces = {}
    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['angle', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
        for angle in sorted(position_map.keys()):
            print(f"Measuring passive forces at {angle}°...")
            pose = position_map[angle]
            arm.set_position(*pose[:3], roll=pose[3], pitch=pose[4], yaw=pose[5], speed=30, wait=True)
            time.sleep(0.5)
            raw_force = np.array(arm.ft_ext_force)
            transformed_force = transform_wrench(raw_force, r_offset)
            passive_forces[angle] = transformed_force
            writer.writerow([angle] + list(transformed_force))
    
    print("\nReturning to starting position after passive force scan...")
    start_pose = position_map[0]
    arm.set_position(
        x=start_pose[0],
        y=start_pose[1],
        z=start_pose[2],
        roll=start_pose[3],
        pitch=start_pose[4],
        yaw=start_pose[5],
        speed=30,
        wait=True
    )
    arm.set_gripper_enable(1)
    arm.set_gripper_position(500)
    time.sleep(5)
    arm.set_gripper_position(119)
    return passive_forces

def force_control_6d_broyden(arm, r_offset, F_target, initial_pose=None, max_iterations=150, timeout=20):
    """
    Force control using Broyden's method with a full 6x6 Jacobian update.
    Ensures final force is re-read after last move (timeout or success).
    """
    threshold = 0.1  # Convergence threshold for force error
    alpha = 0.3      # Initial guess for Jacobian scaling
    success = False  # Track whether force goal was achieved
    start_time = time.time()  # Track total time for timeout

    time.sleep(0.5)  # Brief pause before starting control

    # If an initial pose is given, move robot to that pose first
    if initial_pose is not None:
        arm.set_position(*initial_pose[:3],  # x, y, z
                         roll=initial_pose[3], pitch=initial_pose[4], yaw=initial_pose[5],  # orientation
                         speed=40, wait=True)  # low speed for control, wait for completion
        time.sleep(1)  # Let system stabilize

    # Initialize a 6x6 Jacobian with small identity scaled by alpha
    J = alpha * np.identity(6)
    J_inv = np.linalg.inv(J)  # Pre-compute its inverse

    # Get current robot pose
    ret = arm.get_position()
    if ret[0] != 0:
        print("Failed to get initial pose.")
        return False, None, None, False, []

    pose = np.array(ret[1])  # Extract pose from SDK
    raw_force = np.array(arm.ft_ext_force)  # Read initial raw force
    F_curr = transform_wrench(raw_force, r_offset)  # Transform force to TCP frame

    iteration_data = []  # Store logs of each iteration

    for i in range(max_iterations):
        F_error = F_target - F_curr  # Compute current force error

        # If force error is small enough, we've succeeded
        if np.all(np.abs(F_error) < threshold):
            success = True
            break

        # Compute change in pose needed to reduce force error
        delta_pose = J_inv @ F_error
        pose_new = pose + delta_pose  # Proposed new pose

        # Send robot to the new pose
        arm.set_position(x=pose_new[0], y=pose_new[1], z=pose_new[2],
                         roll=pose_new[3], pitch=pose_new[4], yaw=pose_new[5],
                         speed=3, wait=True)
        time.sleep(1)  # Allow time for robot to settle

        # Read new force after moving
        raw_force_new = np.array(arm.ft_ext_force)
        F_new = transform_wrench(raw_force_new, r_offset)

        # Compute deltas for Broyden update
        y = F_new - F_curr  # Change in force
        s = pose_new - pose  # Change in pose

        # Broyden's update to Jacobian (approximate Jacobian correction)
        y_hat = J @ s  # Predicted ΔF
        if s.T @ y != 0:  # Avoid divide-by-zero
            J += np.outer((y - y_hat), s) / (s @ s)

        # Recompute inverse of updated Jacobian
        J_inv = np.linalg.inv(J)

        # Log iteration data for later analysis
        iteration_data.append({
            'iteration': i,
            'pose': pose.tolist(),
            'force': F_curr.tolist(),
            'error': F_error.tolist()
        })

        # Update for next iteration
        pose = pose_new
        F_curr = F_new

        # Stop if we’ve timed out
        if time.time() - start_time > timeout:
            print("Force control timed out.")
            break

    # 🔄 Final force reading to ensure up-to-date value is returned
    raw_force_final = np.array(arm.ft_ext_force)
    F_final = transform_wrench(raw_force_final, r_offset)

    is_safe = is_force_safe(F_final)  # Safety check for final force
    return success, pose.tolist(), F_final.tolist(), is_safe, iteration_data


def generate_force_validated_map(arm, position_map, transformation_matrix, Of, starting_point,
                                 target_forces=None, passive_forces=None, output_csv_file="force_validated_positions.csv"):
    r_offset = [0.009, 0.0, 0.0]
    force_validated_map = {}

    # Default to 0-force targets if none provided
    if target_forces is None:
        target_forces = {angle: np.zeros(6) for angle in position_map.keys()}

    # Setup CSV file
    with open(output_csv_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['angle', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz', 'is_safe'])

    # Iterate through each angle
    for angle in sorted(position_map.keys()):
        if angle not in target_forces:
            continue

        print(f"\n🔄 Processing angle {angle}°...")

        target_force = target_forces[angle]
        initial_pose = position_map[angle]
        print(f"🔄 Moving to initial guess for angle {angle}°: {initial_pose}")

        # Add passive force if available
        F_passive = passive_forces.get(angle, np.zeros(6)) if passive_forces else np.zeros(6)

        # Perform force control starting from initial guess
        success, tcp_pose, final_force, is_safe, _ = force_control_6d_broyden(
            arm,
            r_offset=r_offset,
            F_target=target_force + F_passive,
            initial_pose=initial_pose
        )

        # Choose fallback if needed
        final_pose = tcp_pose if (success and tcp_pose is not None) else initial_pose
        force_validated_map[angle] = final_pose

        # 📝 Log the final pose and force (even on failure)
        with open(output_csv_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                angle,
                final_pose[0], final_pose[1], final_pose[2],
                final_pose[3], final_pose[4], final_pose[5],
                *(final_force if final_force is not None else np.zeros(6)),
                is_safe
            ])

        print(f"✅ {'Success' if success else 'Timeout'} - Safe: {is_safe}")
        time.sleep(3)  # ✅ Wait briefly before moving to the next angle

    return force_validated_map


# Then in your example_usage() function:
# measure passive forces and generate validated map
# r_offset = [0.009, 0.0, 0.0]
# passive_forces = measure_passive_forces(arm, position_map, r_offset)
# force_validated_map = generate_force_validated_map(arm, position_map, transformation_matrix, Of, starting_point, passive_forces=passive_forces)
USE_SAVED_PASSIVE_FORCES = False  # Set to True to re-measure; False to load from file
passive_force_file = "passive_forces.csv"
def example_usage():
    # Initialize global variable

    current_knee_angle = 0.0
    point_spacing = 25.4  # Distance between points in mm
    starting_point = np.array([597, -31.4, 336])  # Starting point (this is not the real one , this is just used for testing)

    # Create coordinate system points
    p2 = starting_point
    p1 = p2 + np.array([-point_spacing / 2, 0, 0])
    p3 = p2 + np.array([ point_spacing / 2, 0, 0])

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

# Setting hard-coded anatomical reference points for knee coordinate system
    A = np.array([620.4453110694885, 47.15324938297272,171.99364304542542])  # Point A on knee
    B = np.array([626.9657611846924, -29.983650892972946, 171.94780707359314])  # Point B on knee
    C = np.array([626.97, 3.73791, 88.1255])  # Point C on knee
    D = np.array([626.97, 3.73, 40])  # Point D on knee

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

    """Example showing how to use force control with your existing setup"""
    port = '192.168.1.197'
    arm = XArmAPI(port)
    # Enable force torque sensor
    arm.ft_sensor_enable(1)
    time.sleep(0.5)
    arm.ft_sensor_set_zero()
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    
    '''# Your existing variables
    ret = arm.get_position()
    if ret[0] == 0:
        starting_point = np.array(ret[1][:3])  # Get x, y, z only
    else:
        raise RuntimeError("Failed to get current robot position")'''
    
    arm.set_position(
        x=starting_point[0],
        y=starting_point[1],
        z=536,
        roll=-180,
        pitch=0,
        yaw=0,
        speed=80,
        is_radian=False,
        wait=True
    )
    

    
    # Get your original position map using your existing function
    position_map = initialize_flexion_extension(arm, transformation_matrix, Of, starting_point)
    r_offset = [0.009, 0.0, 0.0]  # Offset from sensor to TCP
    #passive_forces = measure_passive_forces(arm, position_map, r_offset)
    if USE_SAVED_PASSIVE_FORCES:
        passive_forces = measure_passive_forces(arm, position_map, r_offset)
    else:
        passive_forces = {}
    try:
        with open(passive_force_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                angle = int(row['angle'])
                force = np.array([
                    float(row['Fx']), float(row['Fy']), float(row['Fz']),
                    float(row['Mx']), float(row['My']), float(row['Mz'])
                ])
                passive_forces[angle] = force
        print("Loaded passive forces from file.")
    except FileNotFoundError:
        print("Passive force file not found. Set USE_SAVED_PASSIVE_FORCES = True to generate it.")
        return  # Exit early if passive data is required
    


    #target force [Fx, Fy, Fz, Mx, My, Mz]
    F_target = np.array([0, -3, 0, 0, 0, 0])

    target_forces = {angle: F_target for angle in position_map.keys()}
    #You likely want to apply this target force at every flexion angle (e.g., during ACL or tibial loading simulation).

    
    # Generate a force-validated map (with zero forces by default)
    force_validated_map = generate_force_validated_map(
    arm, position_map, transformation_matrix, Of, starting_point,
    target_forces=target_forces,
    passive_forces=passive_forces
    )

    
    # Now you can use the force_validated_map instead of the original position_map
    # for more accurate force-controlled motions
    
    arm.disconnect()

if __name__ == "__main__":
    example_usage()

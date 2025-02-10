import numpy as np
import sys
import os
import time
import math
from xarm.x3 import XArm, Studio
from xarm.wrapper import XArmAPI

# Gripper (Load Cell) Points
point_spacing = 25.4  # Distance between points in mm
starting_point = np.array([631.6240429878235, -7.514990400522947, 308.86101722717285])  # Starting point (center of the gripper)
u1 = starting_point + np.array([-point_spacing / 2, 0, 0])  # Point 1, left of center
u2 = starting_point  # Center point
u3 = starting_point + np.array([point_spacing / 2, 0, 0])  # Point 2, right of center

print(f"Gripper Points:\nPoint 1 (u1): {u1}\nCenter (u2): {u2}\nPoint 2 (u3): {u3}")

# Load Cell (Gripper) Coordinate System
Ou = u2  # Center of the gripper is the starting point
x_gripper = u3 - u1  # X-axis: vector between the outer points
y_gripper = np.array([0, 1, 0])  # Assume gripper's Y-axis is global Y for simplicity
z_gripper = np.cross(x_gripper, y_gripper)  # Z-axis: orthogonal to X and Y

# Normalize the gripper axes
x_gripper = x_gripper / np.linalg.norm(x_gripper)
y_gripper = y_gripper / np.linalg.norm(y_gripper)
z_gripper = z_gripper / np.linalg.norm(z_gripper)

print("\nGripper (Load Cell) Coordinate System:")
print(f"Center (Ou): {Ou}")
print(f"X-axis: {x_gripper}")
print(f"Y-axis: {y_gripper}")
print(f"Z-axis: {z_gripper}")

# Knee Coordinate System
A = np.array([620.4453110694885, 47.15324938297272, 171.99364304542542])   # Femur point A
B = np.array([626.9657611846924, -29.983650892972946, 171.94780707359314])   # Femur point B
C = np.array([608.878, 5.38872, 104.393])   # Tibia point C
D = np.array([617.938, 4.60259, 80.3833])   # Tibia point D

Of = (A + B) / 2  # Femur center
Ot = Of + (D - C)  # Tibia center
x_knee = Of - Ot   # X-axis
y_knee = A - B     # Y-axis
z_knee = np.cross(x_knee, y_knee)  # Z-axis (cross-product)

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
R_knee_to_gripper = np.array([x_gripper, y_gripper, z_gripper]).T @ np.array([x_knee, y_knee, z_knee])
translation = Ou - Of  # Gripper center to femur center
transformation_matrix = np.eye(4)  # Identity matrix
transformation_matrix[:3, :3] = R_knee_to_gripper  # Top-left 3x3 is rotation
transformation_matrix[:3, 3] = translation  # Top-right 3x1 is translation vector

print("\nTransformation Matrix (Knee Relative to Gripper):")
print(transformation_matrix)

def test_rotations(angle_degrees):
    angle_radians = np.radians(angle_degrees)
    
    # Create rotation matrix for knee flexion around robot's Y-axis
    rotation = np.array([
        [np.cos(angle_radians), -np.sin(angle_radians), 0],
        [np.sin(angle_radians),  np.cos(angle_radians), 0],
        [0,                      0,                     1]
    ])
    
    Of_homog = np.append(Of, 1)
    pivot_knee = np.linalg.inv(transformation_matrix) @ Of_homog
    
    head_homog = np.append(starting_point, 1)
    head_knee = np.linalg.inv(transformation_matrix) @ head_homog
    
    pivot_to_head = head_knee[:3] - pivot_knee[:3]
    
    rotated_vector = rotation @ pivot_to_head
    
    new_head_knee = pivot_knee[:3] + rotated_vector
    
    new_head_homog = np.append(new_head_knee, 1)
    new_head_global = transformation_matrix @ new_head_homog
    
    print(f"\nAngle: {angle_degrees} degrees")
    print(f"Pivot point (knee): {Of}")
    print(f"Original gripper/femur head position: {starting_point}")
    print(f"New gripper/femur head position: {new_head_global[:3]}")
    
    return new_head_global[:3]

def test_different_movements(arm, test_angles):
    try:
        # Store starting position
        start_pos = test_rotations(0)  # Get position at 0 degrees
        
        print("\n1. Testing set_position (point-to-point)")
        for angle in test_angles:
            pos = test_rotations(angle)
            print(f"\nMoving to {angle} degrees using set_position")
            arm.set_position(
                x=float(pos[0]), 
                y=float(pos[1]), 
                z=float(pos[2]),
                wait=True
            )
            time.sleep(2)
        
        # Return to start
        print("\nReturning to start position")
        arm.set_position(
            x=float(start_pos[0]),
            y=float(start_pos[1]),
            z=float(start_pos[2]),
            wait=True
        )
        time.sleep(2)
        
        '''# Switch to servo mode
        arm.set_mode(1)
        arm.set_state(0)
        time.sleep(0.1)  # Allow mode switch to complete
        
        # Execute rotational movements
        while arm.connected and arm.state != 4:
            for angle in test_angles:
                pos = test_rotations(angle)
                print(f"\nMoving to {angle} degrees using set_servo_cartesian")
                ret = arm.set_servo_cartesian(
                    [
                        float(pos[0]), 
                        float(pos[1]), 
                        float(pos[2]),
                        -180,  # roll
                        0,     # pitch
                        0      # yaw
                    ],
                    speed=10,   # Adjust speed as needed
                    mvacc=1   # Adjust acceleration as needed
                )
                print(f'set_servo_cartesian, ret={ret}')
                time.sleep(1)  # Small delay for smoother motion
            
            # Break after one complete cycle
            break
        
        # Switch back to position mode
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.1)
            
        # Return to start
        print("\nReturning to start position")
        arm.set_position(
            x=float(start_pos[0]),
            y=float(start_pos[1]),
            z=float(start_pos[2]),
            wait=True
        )
        time.sleep(2)'''
        
        print("\n3. Testing arc line movement")
        paths = []
        for angle in test_angles:
            pos = test_rotations(angle)
            paths.append([
                float(pos[0]), 
                float(pos[1]), 
                float(pos[2]), 
        -180, 0, 0  # roll, pitch, yaw
    ])

# First move to initial position
        '''print("Moving to start position")
        arm.set_position(*paths[0], wait=True)

# Set pause time between movements
        arm.set_pause_time(0.2)

# Move through all positions
        for path in paths:
            ret = arm.set_position(*path[:6], radius=0, wait=True, speed=50)
            if ret < 0:
                print('set_position failed, ret={}'.format(ret))
                break
            time.sleep(1)'''
        arm.move_arc_lines(paths, speed=50, times=3, wait=True)
# Return to start
        print("\nReturning to start position")
        arm.set_position(
        x=float(start_pos[0]),
        y=float(start_pos[1]),
        z=float(start_pos[2]),
        wait=True
)
        time.sleep(2)
        
        '''print("\n4. Testing move_circle")
        pos_start = test_rotations(test_angles[0])
        pos_end = test_rotations(test_angles[-1])
        print("\nMoving in circular path")
        arm.move_circle(
            pose1=[float(pos_start[0]), float(pos_start[1]), float(pos_start[2]), 0, 0, 0],
            pose2=[float(pos_end[0]), float(pos_end[1]), float(pos_end[2]), 0, 0, 0],
            percent=50,
            wait=True
        )
        
        # Return to start
        print("\nReturning to start position")
        arm.set_position(
            x=float(start_pos[0]),
            y=float(start_pos[1]),
            z=float(start_pos[2]),
            wait=True
        )
        time.sleep(2)
        '''
        print("\n5. Testing set_servo_angle")
        code, angles = arm.get_servo_angle()
        if code == 0:
            print("\nMoving using joint angles")
            for i in range(len(angles)):
                modified_angles = angles.copy()
                modified_angles[i] += 10
                arm.set_servo_angle(angle=modified_angles, wait=True)
                time.sleep(1)
                arm.set_servo_angle(angle=angles, wait=True)
                time.sleep(1)
        
        
        print("\nReturning to final start position")
        arm.set_position(
            x=float(start_pos[0]),
            y=float(start_pos[1]),
            z=float(start_pos[2]),
            wait=True
        )
                
    except Exception as e:
        print(f"Error occurred: {e}")
        arm.emergency_stop()

# Test angles
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

test_angles = [0, 30, 45, 90]
'''for angle in test_angles:
    pos = test_rotations(angle)'''
try:
    '''for angle in test_angles:
        pos = test_rotations(angle)
        print(f"\nMoving to position for {angle} degrees:")
        # Move to the calculated position
        code = arm.set_position(
            x=float(pos[0]), 
            y=float(pos[1]), 
            z=float(pos[2]),
            wait=True  # Wait for movement to complete
        )
        if code != 0:
            print(f"Movement failed with error code: {code}")
        time.sleep(2)  # Wait between movements'''

    test_different_movements(arm, test_angles)
        
finally:
    # Always clean up
    arm.disconnect()

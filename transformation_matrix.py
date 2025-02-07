import numpy as np
import sys
import os
import time
import math
from xarm.x3 import XArm, Studio
from xarm.wrapper import XArmAPI
# Gripper (Load Cell) Points
point_spacing = 25.4  # Distance between points in mm
starting_point = np.array([631.624, -7.51499 , 308.861])  # Starting point (center of the gripper)
u1 = starting_point + np.array([-point_spacing / 2, 0, 0])  # Point 1, left of center
u2 = starting_point  # Center point
u3 = starting_point + np.array([point_spacing / 2, 0, 0])  # Point 2, right of center

print(f"Gripper Points:\nPoint 1 (u1): {u1}\nCenter (u2): {u2}\nPoint 2 (u3): {u3}")

# **Load Cell (Gripper) Coordinate System**
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

# **Knee Coordinate System**
# Example femur and tibia points (replace with actual digitized points)
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

# **Compute Transformation Matrix**
# Rotation matrix: Align knee axes to gripper axes
R_knee_to_gripper = np.array([x_gripper, y_gripper, z_gripper]).T @ np.array([x_knee, y_knee, z_knee])

# Translation vector: Move knee relative to gripper center
translation = Ou - Of  # Gripper center to femur center

# Transformation matrix (4x4, combining rotation and translation)
transformation_matrix = np.eye(4)  # Identity matrix
transformation_matrix[:3, :3] = R_knee_to_gripper  # Top-left 3x3 is rotation
transformation_matrix[:3, 3] = translation  # Top-right 3x1 is translation vector

print("\nTransformation Matrix (Knee Relative to Gripper):")
print(transformation_matrix)

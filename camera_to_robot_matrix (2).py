
import numpy as np

def camera_to_robot_transform(marker_sets_camera: np.ndarray, robot_positions: np.ndarray) -> np.ndarray:
    """
    Computes the transformation matrix from camera frame to robot frame
    using marker displacements observed from camera and known robot motion.

    Parameters:
    - marker_sets_camera: (3, 2, 3) NumPy array. 3 steps, 2 markers per step, 3D coords
    - robot_positions: (3, 3) NumPy array. 3 known robot TCP positions at each step.

    Returns:
    - T_cam_to_robot: 4x4 transformation matrix as a NumPy array
    """

    if marker_sets_camera.shape != (3, 2, 3):
        raise ValueError("Expected marker_sets_camera to be of shape (3, 2, 3)")
    if robot_positions.shape != (3, 3):
        raise ValueError("Expected robot_positions to be of shape (3, 3)")

    # Compute the midpoint of the 2 markers at each step
    marker_centers = marker_sets_camera.mean(axis=1)

    # Center the points
    centroid_cam = np.mean(marker_centers, axis=0)
    centroid_robot = np.mean(robot_positions, axis=0)

    cam_centered = marker_centers - centroid_cam
    robot_centered = robot_positions - centroid_robot

    # Compute rotation matrix
    H = cam_centered.T @ robot_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Compute translation
    t = centroid_robot - R @ centroid_cam

    # Assemble transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

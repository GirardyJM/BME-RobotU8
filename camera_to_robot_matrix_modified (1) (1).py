
import numpy as np

def camera_to_robot_transform_from_raw_inputs(
    tcp_initial,
    marker1_initial,
    marker2_initial,
    marker1_after_x,
    marker2_after_x,
    marker1_after_y,
    marker2_after_y,
    marker1_after_z,
    marker2_after_z
):
    """
    Computes the transformation matrix from Orbbec camera frame to robot frame.

    Inputs:
    - tcp_initial: (3,) numpy array
    - marker1_initial, marker2_initial: (3,) numpy arrays for initial marker positions
    - marker1_after_x, marker2_after_x: after +10mm in X
    - marker1_after_y, marker2_after_y: after +10mm in Y (cumulative from X)
    - marker1_after_z, marker2_after_z: after +10mm in Z (cumulative from X and Y)

    Returns:
    - 4x4 transformation matrix (camera → robot)
    """

    # Build robot positions based on known displacements from tcp_initial
    robot_positions = np.array([
        tcp_initial,
        tcp_initial + np.array([10.0, 0.0, 0.0]),
        tcp_initial + np.array([10.0, 10.0, 0.0])
    ])

    # Organize camera marker data (only initial, X, and Y used)
    marker_sets_camera = np.array([
        [marker1_initial, marker2_initial],
        [marker1_after_x, marker2_after_x],
        [marker1_after_y, marker2_after_y]
    ])  # Shape: (3, 2, 3)

    # Compute the midpoint of the 2 markers at each step
    marker_centers = marker_sets_camera.mean(axis=1)

    # Compute centroids
    centroid_cam = np.mean(marker_centers, axis=0)
    centroid_robot = np.mean(robot_positions, axis=0)

    # Centered vectors
    cam_centered = marker_centers - centroid_cam
    robot_centered = robot_positions - centroid_robot

    # SVD for rotation
    H = cam_centered.T @ robot_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_robot - R @ centroid_cam

    # Build transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

import numpy as np
import re
#from scipy.spatial.transform import Rotation as R

def icp_transform_orbbec(source_points, target_points):
    """
    Perform an ICP-based rigid transformation alignment from source_points to target_points.

    Parameters:
    - source_points: np.ndarray of shape (N, 3), the original 3D coordinates (e.g., from 3D scanner)
    - target_points: np.ndarray of shape (N, 3), the observed 3D coordinates (e.g., from Orbbec)

    Returns:
    - transformation_matrix: np.ndarray of shape (4, 4), the homogeneous transformation matrix
      that maps source_points to target_points
    """
    source_points = np.asarray(source_points)
    target_points = np.asarray(target_points)

    assert source_points.shape == target_points.shape, "Source and target must have the same shape"
    assert source_points.shape[1] == 3, "Points must be 3D coordinates"

    # Compute centroids
    source_mean = np.mean(source_points, axis=0)
    target_mean = np.mean(target_points, axis=0)

    # Center the points
    source_centered = source_points - source_mean
    target_centered = target_points - target_mean

    # Compute rotation using SVD
    H = source_centered.T @ target_centered
    U, S, Vt = np.linalg.svd(H)
    R_icp = Vt.T @ U.T

    # Correct improper rotation (reflection case)
    if np.linalg.det(R_icp) < 0:
        Vt[-1, :] *= -1
        R_icp = Vt.T @ U.T

    # Compute translation
    t_icp = target_mean - R_icp @ source_mean

    # Construct homogeneous transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R_icp
    transformation_matrix[:3, 3] = t_icp

    return transformation_matrix


def str_to_np_array_custom(input_str):
    """
    Converts a string like "(451, 185) = 40" to a NumPy array.
    Outputs: np.array([451.0, 185.0, 40.0])
    """

    try:
        # Use regex to extract numbers
        numbers = re.findall(r"[-+]?\d*\.?\d+", input_str)
        float_numbers = [float(num) for num in numbers]
        return np.array(float_numbers)
    except Exception as e:
        return np.array([0])

def parse_blender_string(blender_str):
    """
    Converts a Blender-style string like:
    "-2.675889730453491, -3.0188241004943848, 2.466001510620117"
    into a NumPy array.
    """
    try:
        coords = np.fromstring(blender_str, sep=',')
        if coords.shape[0] != 3:
            raise ValueError("Input must contain exactly three values.")
        return coords
    except Exception as e:
        #print("Error parsing Blender string:", e)
        return None


def camera_to_robot_transform_from_raw_inputs(
    tcp_initial,
    marker1_initial,
    marker2_initial,
    marker1_after_x,
    marker2_after_x,
    marker1_after_y,
    marker2_after_y,

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

def transform_icp_to_robot(icp_points, T_camera_to_robot):
    '''
    Applies a 4x4 transformation matrix to a 9x3 ICP output matrix.

    Parameters:
    - icp_points: (9, 3) array of points in camera space
    - T_camera_to_robot: (4, 4) transformation matrix from camera to robot

    Returns:
    - (9, 3) array of transformed points in robot space
    '''
    icp_points = np.asarray(icp_points)
    T_camera_to_robot = np.asarray(T_camera_to_robot)
    if icp_points.shape != (9, 3):
        raise ValueError("Expected icp_points to be 9x3")
    if T_camera_to_robot.shape != (4, 4):
        raise ValueError("Expected T_camera_to_robot to be 4x4")

    # Step 1: Convert to homogeneous coordinates (9x4)
    icp_points_hom = np.hstack([icp_points, np.ones((9, 1))])

    # Step 2: Apply the transformation
    icp_points_hom=np.ascontiguousarray(icp_points_hom)
    transformed_hom = (T_camera_to_robot @ icp_points_hom.T).T  # Result is 9x4

    # Step 3: Drop the last column to return 9x3
    transformed_robot_points = transformed_hom[:, :3]
    transformed_robot_points = np.ascontiguousarray(transformed_hom[:, :3].copy())
    transformed_robot_points[:, 2] *= -1
    return np.ascontiguousarray(transformed_robot_points)
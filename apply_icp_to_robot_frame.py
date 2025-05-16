
import numpy as np

def transform_icp_to_robot(icp_points: np.ndarray, T_camera_to_robot: np.ndarray) -> np.ndarray:
    '''
    Applies a 4x4 transformation matrix to a 9x3 ICP output matrix.

    Parameters:
    - icp_points: (9, 3) array of points in camera space
    - T_camera_to_robot: (4, 4) transformation matrix from camera to robot

    Returns:
    - (9, 3) array of transformed points in robot space
    '''
    if icp_points.shape != (9, 3):
        raise ValueError("Expected icp_points to be 9x3")
    if T_camera_to_robot.shape != (4, 4):
        raise ValueError("Expected T_camera_to_robot to be 4x4")

    # Step 1: Convert to homogeneous coordinates (9x4)
    icp_points_hom = np.hstack([icp_points, np.ones((9, 1))])

    # Step 2: Apply the transformation
    transformed_hom = (T_camera_to_robot @ icp_points_hom.T).T  # Result is 9x4

    # Step 3: Drop the last column to return 9x3
    transformed_robot_points = transformed_hom[:, :3]

    return transformed_robot_points

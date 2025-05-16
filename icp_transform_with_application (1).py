
import numpy as np
import open3d as o3d

def icp_transform_orbbec(scanner_points: np.ndarray, orbbec_points: np.ndarray) -> np.ndarray:
    '''
    Perform ICP registration to align scanner_points to orbbec_points.
    
    Parameters:
    - scanner_points: (9, 3) array from 3D scanner
    - orbbec_points: (9, 3) array from Orbbec camera

    Returns:
    - 4x4 transformation matrix that maps scanner points into Orbbec camera space
    '''
    # Create Open3D point clouds
    source = o3d.geometry.PointCloud()
    target = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(scanner_points)
    target.points = o3d.utility.Vector3dVector(orbbec_points)

    # Initial guess (identity)
    trans_init = np.eye(4)
    threshold = 50.0  # max correspondence distance

    # ICP registration
    reg_icp = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    return reg_icp.transformation


def apply_icp_and_transform_points(scanner_points: np.ndarray, orbbec_points: np.ndarray) -> np.ndarray:
    '''
    Uses ICP to compute the transformation from scanner to camera space,
    then applies that transformation to the scanner points.

    Parameters:
    - scanner_points: (9, 3) NumPy array of 3D points from the scanner
    - orbbec_points: (9, 3) NumPy array of 3D points from the Orbbec camera (same landmarks)

    Returns:
    - transformed_points: (9, 3) NumPy array of scanner points expressed in Orbbec camera coordinates
    '''
    T_icp = icp_transform_orbbec(scanner_points, orbbec_points)
    scanner_hom = np.hstack([scanner_points, np.ones((scanner_points.shape[0], 1))])
    transformed_hom = (T_icp @ scanner_hom.T).T
    transformed_points = transformed_hom[:, :3]
    return transformed_points

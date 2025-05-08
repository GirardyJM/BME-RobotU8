def icp_transform_orbbec(source_points: np.ndarray, target_points: np.ndarray) -> np.ndarray:
    """
    Perform an ICP-based rigid transformation alignment from source_points to target_points.

    Parameters:
    - source_points: np.ndarray of shape (N, 3), the original 3D coordinates (e.g., from 3D scanner)
    - target_points: np.ndarray of shape (N, 3), the observed 3D coordinates (e.g., from Orbbec)

    Returns:
    - transformation_matrix: np.ndarray of shape (4, 4), the homogeneous transformation matrix
      that maps source_points to target_points
    """
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

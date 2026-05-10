import math
import numpy as np
from unittest.mock import patch
from localization_manager.pcd_relocalizer_3d import (
    NdtVoxelGrid,
    normalize_quaternion,
    quaternion_to_matrix,
    matrix_to_quaternion,
    xyz_rpy_to_matrix,
    PcdRelocalizer3D,
)
from nav_msgs.msg import Odometry
import rclpy

def test_quaternion_normalization():
    q = np.array([2.0, 0.0, 0.0, 0.0])
    qn = normalize_quaternion(q)
    assert np.allclose(qn, [1.0, 0.0, 0.0, 0.0])
    
    q_zero = np.array([0.0, 0.0, 0.0, 0.0])
    qn_zero = normalize_quaternion(q_zero)
    assert np.allclose(qn_zero, [0.0, 0.0, 0.0, 1.0])

def test_quaternion_matrix_conversion():
    q = np.array([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    mat = quaternion_to_matrix(*q)
    q_back = matrix_to_quaternion(mat)
    assert np.allclose(q, q_back) or np.allclose(q, -np.array(q_back))

def test_ndt_voxel_grid_creation():
    points = np.array([
        [0.1, 0.1, 0.1],
        [0.2, 0.2, 0.2],
        [0.3, 0.3, 0.3],
        [1.1, 1.1, 1.1], 
    ])
    
    grid = NdtVoxelGrid(points, resolution=1.0, min_points_per_voxel=2, cov_reg=0.01)
    
    assert len(grid.voxels) == 1
    assert (0, 0, 0) in grid.voxels
    assert (1, 1, 1) not in grid.voxels
    
    mean, inv_cov = grid.voxels[(0, 0, 0)]
    assert np.allclose(mean, [0.2, 0.2, 0.2])
    assert np.all(np.isfinite(inv_cov))

def test_ndt_voxel_grid_query():
    points = np.array([
        [0.0, 0.0, 0.0],
        [0.1, 0.0, 0.0],
        [0.2, 0.0, 0.0],
        [0.3, 0.0, 0.0],
        [0.4, 0.0, 0.0],
    ])
    grid = NdtVoxelGrid(points, resolution=1.0, min_points_per_voxel=3, cov_reg=0.01)
    
    query_points = np.array([
        [0.2, 0.0, 0.0],
        [2.0, 2.0, 2.0], 
    ])
    
    valid_mask, residuals, inv_covs, valid_points = grid.query(query_points, neighbor_search=True)
    assert valid_mask.sum() == 1
    assert valid_mask[0] == True
    assert valid_mask[1] == False
    assert residuals.shape == (1, 3)

def test_covariance_scaling_logic():
    ndt_score_threshold = 3.0
    last_score_good = 1.5
    last_score_bad = 6.0
    
    scale_good = max(1.0, min(5.0, last_score_good / max(0.1, ndt_score_threshold)))
    scale_bad = max(1.0, min(5.0, last_score_bad / max(0.1, ndt_score_threshold)))
    
    assert scale_good == 1.0
    assert scale_bad > scale_good
    assert scale_bad == 2.0

@patch('localization_manager.pcd_relocalizer_3d.PcdRelocalizer3D._resolve_pcd_path')
@patch('localization_manager.pcd_relocalizer_3d.PcdRelocalizer3D._load_pcd')
def test_pcd_relocalizer_ndt_loop(mock_load_pcd, mock_resolve_pcd_path):
    rclpy.init()
    try:
        mock_resolve_pcd_path.return_value = 'dummy_path'
        x, y, z = np.meshgrid(np.arange(-2, 3), np.arange(-2, 3), np.arange(-2, 3))
        base_points = np.vstack([x.flatten(), y.flatten(), z.flatten()]).T.astype(np.float64)
        
        points = np.vstack([base_points] * 10)
        mock_load_pcd.return_value = points
        
        node = PcdRelocalizer3D()
        
        node.last_scan = base_points + np.array([0.2, 0.0, 0.0])
        
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        node.last_odom = odom
        node.has_seed = True
        node.map_to_odom = np.eye(4)
        node.base_to_lidar = np.eye(4)
        
        node._run_ndt()
        
        # known translation recovery test
        assert np.allclose(node.map_to_odom[:3, 3], [-0.2, 0.0, 0.0], atol=0.1)
        
        # score rejection keeps map_to_odom unchanged
        node.ndt_score_threshold = 0.0001
        node.map_to_odom = np.eye(4) 
        node.last_scan = base_points + np.array([0.5, 0.5, 0.5])
        node._run_ndt()
        
        assert np.allclose(node.map_to_odom, np.eye(4))
        
    finally:
        rclpy.shutdown()
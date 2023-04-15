import open3d as o3d
import numpy as np
import cupy as cp


__all__ = ['VELODYNE_INDEXDER', 'sph_pcd_to_cart_pcd', 'depth_to_sph_pts']


VELODYNE_INDEXDER = {
    0:15,
    2:14,
    4:13,
    6:12,
    8:11,
    10:10,
    12:9,
    14:8,
    1:7,
    3:6,
    5:5,
    7:4,
    9:3,
    11:2,
    13:1,
    15:0,
}


def sph_pcd_to_cart_pcd(sph_pcd):
    sph_pcd[:,1] = cp.radians(sph_pcd[:,1])
    sph_pcd[:,2] = cp.radians(sph_pcd[:,2])

    # Convert spherical coordinates to Cartesian coordinates
    x = sph_pcd[:, 0] * cp.cos(sph_pcd[:, 1]) * cp.cos(sph_pcd[:, 2])
    y = sph_pcd[:, 0] * cp.cos(sph_pcd[:, 1]) * cp.sin(sph_pcd[:, 2])
    z = sph_pcd[:, 0] * cp.sin(sph_pcd[:, 1])
    
    cart_pts = cp.asarray([x, y, z]).T

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cart_pts.get())
    return pcd


def depth_to_sph_pts(depth):
    # Make sure input depth array is a CuPy array
    depth = cp.array(depth)
    
    # get the shape of the input array
    m, n = depth.shape
    azimuth_const = 100/n
    polar_const = 30/m

    # Create a grid of row and col indices
    row_indices, col_indices = cp.meshgrid(cp.arange(m), cp.arange(n), indexing='ij')

    # Calculate polar and azimuth angles
    polar_angles = row_indices * polar_const
    azimuth_angles = col_indices * azimuth_const

    # Stack the depth, polar_angles, and azimuth_angles along the last dimension
    pts = cp.stack((depth, polar_angles, azimuth_angles), axis=-1)

    # Reshape the pts array to the desired output shape (m * n, 3)
    pts = pts.reshape(m * n, 3)

    return pts
import open3d as o3d
import numpy as np


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
    sph_pcd[:,1] = np.radians(sph_pcd[:,1])
    sph_pcd[:,2] = np.radians(sph_pcd[:,2])

    # Convert spherical coordinates to Cartesian coordinates
    x = sph_pcd[:, 0] * np.cos(sph_pcd[:, 1]) * np.cos(sph_pcd[:, 2])
    y = sph_pcd[:, 0] * np.cos(sph_pcd[:, 1]) * np.sin(sph_pcd[:, 2])
    z = sph_pcd[:, 0] * np.sin(sph_pcd[:, 1])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray([x, y, z]).T)
    return pcd


def depth_to_sph_pts(depth):
    # get the shape of the input arr
    m, n = depth.shape
    azimuth_const = 100/n
    polar_const = 30/m
    
    # create a 3D output arr of size (m * n, 3)
    pts = np.zeros((m * n, 3))
    
    # populate the ptsput arr
    for row in range(m):
        for col in range(n):
            index = row * n + col
            pts[index, 0] = depth[row, col]
            pts[index, 1] = row * polar_const
            pts[index, 2] = col * azimuth_const 
    
    return pts
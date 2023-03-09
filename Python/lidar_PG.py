import pandas as pd
import open3d as o3d
import numpy as np
import csv
import copy
import scipy
import os

ZED_data = '../data/ZED/vga/point_cloud_PLY_3029_376_06-12-2022-19-40-31.ply'
LiDAR_data = '../data/lidar/300/2022-12-10-16-53-36_Velodyne-VLP-16-Data.csv'

os.path.exists(ZED_data)
os.path.exists(LiDAR_data)

# ---------------

pcd_zed = o3d.io.read_point_cloud(ZED_data)
pts_zed = np.asarray(pcd_zed.points)

print(pts_zed)
o3d.visualization.draw_geometries([pcd_zed])

# ---------------

with open(LiDAR_data, newline='') as f:
    rows = list(csv.reader(f, delimiter=',', quotechar='"'))
    pts_lidar = np.zeros(shape=(len(rows) - 1, 3))
    headers = rows.pop(0)
    for index, row in enumerate(rows[1:]):
        # print(index, row[0])
        pts_lidar[index] = row[0:3]

# ---------------

coords = o3d.geometry.TriangleMesh.create_coordinate_frame()
pcd_lidar = o3d.geometry.PointCloud()
pcd_lidar.points = o3d.utility.Vector3dVector(pts_lidar)

# ---------------

# print(pts_lidar)

pcd_lidar_r = copy.deepcopy(pcd_lidar).translate((0, 0, 0))
pcd_lidar_r.rotate(pcd_lidar_r.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0)), center=(0, 0, 0))
# print(np.asarray(pcd_lidar_r.points))

# ---------------

pts_lidar_r = np.asarray(pcd_lidar_r.points)

shape_zed = np.shape(pts_zed)

print(np.fft.fft2(pts_lidar_r))

# shape_lidar = np.shape(pts_lidar_r)
# print(shape_zed[0])
# x = np.array(range(shape_zed[0]))
# y = np.array(range(3))
# pts_lidar_u = scipy.interpolate.interp2d(x, y, pts_lidar_r, kind='linear')

# ---------------

pcd_zed.paint_uniform_color([1, 0.706, 0])
pcd_lidar_r.paint_uniform_color([1, 0, 0])

# ---------------

o3d.visualization.draw_geometries([
    # coords,
    pcd_zed,
    # pcd_lidar,
    pcd_lidar_r
])


# ---------------

#Writing points with rows as the coordinates
p1_t = np.asarray(pcd_lidar.points)
p2_t = np.asarray(pcd_lidar_r.points) #Approx transformation is 90 degree rot over x-axis and +1 in Z axis

print(p1_t)
print(p2_t)

#Take transpose as columns should be the points
p1 = p1_t.transpose()
p2 = p2_t.transpose()

#Calculate centroids
p1_c = np.mean(p1, axis = 1).reshape((-1,1)) #If you don't put reshape then the outcome is 1D with no rows/colums and is interpeted as rowvector in next minus operation, while it should be a column vector
p2_c = np.mean(p2, axis = 1).reshape((-1,1))

#Subtract centroids
q1 = p1-p1_c
q2 = p2-p2_c

#Calculate covariance matrix
H=np.matmul(q1,q2.transpose())

#Calculate singular value decomposition (SVD)
U, X, V_t = np.linalg.svd(H) #the SVD of linalg gives you Vt

#Calculate rotation matrix
R = np.matmul(V_t.transpose(),U.transpose())

assert np.allclose(np.linalg.det(R), 1.0), "Rotation matrix of N-point registration not 1, see paper Arun et al."

#Calculate translation matrix
T = p2_c - np.matmul(R,p1_c)

#Check result
result = T + np.matmul(R,p1)
if np.allclose(result,p2):
    print("transformation is correct!")
else:
    print("transformation is wrong...")

# ---------------
print(type(result), result)


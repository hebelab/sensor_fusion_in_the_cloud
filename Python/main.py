import open3d as o3d
import numpy as np
import cupy as cp
import matplotlib.pyplot as plt
import csv
import time

from utils import *


LiDAR_data = '../data/20221210/lidar/300/2022-12-10-16-53-36_Velodyne-VLP-16-Data.csv'
ZED_data = '../data/20221210/ZED/720/point_cloud_PLY_3029_720_06-12-2022-19-39-57.ply'

ZED_V = 720
ZED_H = 1280

LiDAR_V = 16


def load_lidar(path):
    with open(path, newline='') as f:
        rows = list(csv.reader(f, delimiter=',', quotechar='"'))
        sph_lidar = np.zeros(shape=(len(rows) - 1, 3))
        headers = rows.pop(0)
        for index, row in enumerate(rows[1:]):
            sph_lidar[index] = row[7:10]

    gt_mean = sph_lidar[:, 2].mean()

    sph_lidar[:, 0] = np.array(
        list(map(VELODYNE_INDEXDER.get, sph_lidar[:, 0])))

    mid_cutoff = 18000
    sph_lidar[:, 1] += mid_cutoff

    mask = sph_lidar[:, 1] > 2*mid_cutoff

    sph_lidar[mask, 1] -= 2*mid_cutoff

    # Split the array based on the range of the first column
    sph_lidar_rows = []
    for i in range(LiDAR_V):
        lower = i
        upper = i + 1
        mask = (sph_lidar[:, 0] >= lower) & (sph_lidar[:, 0] < upper)
        sub_array = sph_lidar[mask, 1:]
        sph_lidar_rows.append(sub_array)

    # Convert each sub-array to a 1D array sorted by the first column and using the second column as the value
    sph_lidar_frame = []
    for sub_array in sph_lidar_rows:
        indices = np.argsort(sub_array[:, 0])
        sorted_array = sub_array[indices, 1].tolist()

        row_len = len(sorted_array)
        num_zeros = ZED_H - row_len
        step_size = row_len // (num_zeros + 1)

        # Loop over the array and insert zeros at regular intervals
        for i in range(num_zeros):
            index = (i + 1) * step_size
            sorted_array.insert(index, gt_mean)

        sph_lidar_frame.append(sorted_array)

    return sph_lidar_frame


def load_zed(path):
    def appendSpherical_np(xyz):
        ptsnew = np.hstack((xyz, np.zeros(xyz.shape)))
        xy = xyz[:, 2]**2 + xyz[:, 1]**2
        ptsnew[:, 3] = np.sqrt(xy + xyz[:, 0]**2)
        ptsnew[:, 4] = np.arctan2(xyz[:, 0], np.sqrt(xy))
        ptsnew[:, 5] = np.arctan2(xyz[:, 1], xyz[:, 2])
        return ptsnew

    def map_range_to_interval(value, old_min, old_max, new_min, new_max):
        mapped_value = (value - old_min) * (new_max - new_min) / \
            (old_max - old_min) + new_min
        return int(mapped_value)

    pcd_zed = o3d.io.read_point_cloud(path)
    pts_zed = np.asarray(pcd_zed.points)

    pts_zed_sph = appendSpherical_np(pts_zed)[:, 3:6]
    pts_zed_sph[:, 1:3] = np.degrees(pts_zed_sph[:, 1:3]) + 180

    a = pts_zed_sph[:, 2]

    lower_angle = 20
    upper_angle = 20

    polar_min, polar_max = pts_zed_sph[:, 2].min(
    ) + lower_angle, pts_zed_sph[:, 2].max() - upper_angle

    pts_zed_sph_filt = pts_zed_sph[(a < polar_min) | (a > polar_max)]

    pts_zed_sph_filt[:, 1] *= 14.03
    pts_zed_sph_filt[:, 1] = pts_zed_sph_filt[:, 1]//1 - 1903

    mask = pts_zed_sph_filt[:, 2] > lower_angle

    pts_zed_sph_filt[mask, 2] -= 360
    pts_zed_sph_filt[not mask.all(), 2] += lower_angle

    pts_zed_sph_filt[:, 2] *= 1.6

    v_range = (pts_zed_sph_filt[:, 2].min(), pts_zed_sph_filt[:, 2].max()+1)

    pts_zed_sph_filt[:, 2] = np.vectorize(map_range_to_interval)(
        pts_zed_sph_filt[:, 2], v_range[0], v_range[1], 0, 720)

    bg_depth = pts_zed_sph_filt[:, 0].mean()

    sph_zed_frame = []
    for i in range(0, ZED_V):
        lower = i
        upper = i + 1
        mask = (pts_zed_sph_filt[:, 2] >= lower) & (
            pts_zed_sph_filt[:, 2] < upper)
        sub_array = pts_zed_sph_filt[mask, 0:2]

        row = np.ones(ZED_H)*bg_depth
        row[sub_array[:, 1].astype(int)] = sub_array[:, 0]
        sph_zed_frame.append(row)
    sph_zed_frame = np.asarray(sph_zed_frame)

    return sph_zed_frame


def main():
    start = time.time()
    sph_lidar_frame = load_lidar(LiDAR_data)
    print(time.time() - start)
    
    start = time.time()
    sph_zed_frame = load_zed(ZED_data)
    print(time.time() - start)
    
    plt.imsave('sph_lidar_frame.png', sph_lidar_frame)
    plt.imsave('sph_zed_frame.png', sph_zed_frame)

    m = ZED_V//LiDAR_V
    
    # plt.imsave('pg_frame_init.png', pg_frame_init)

    for threshold in range(1, 100):
        pg_frame_init = sph_zed_frame
        pg_frame_init[::m, :] = sph_lidar_frame
    
        start = time.time()
        pg_frame = pg(cp.asarray(pg_frame_init), m, ncutoff=1, threshold=threshold)
        print(f'\t{threshold}: ', time.time() - start)

    # plt.imsave('pg_frame.png', pg_frame.get())

    start = time.time()
    pg_pts = depth_to_sph_pts(pg_frame)
    print(time.time() - start)
    
    start = time.time()
    pg_pcd = sph_pcd_to_cart_pcd(pg_pts)
    print(time.time() - start)
    
    o3d.visualization.draw_geometries([
        pg_pcd,
    ])


if __name__ == '__main__':
    main()

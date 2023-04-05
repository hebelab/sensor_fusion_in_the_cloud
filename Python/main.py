# %%
import open3d as o3d
import numpy as np
import cupy as cp
import matplotlib.pyplot as plt
import cv2
import csv

# %%
LiDAR_data = '../data/20221210/lidar/300/2022-12-10-16-53-36_Velodyne-VLP-16-Data.csv'
ZED_data = '../data/20221210/ZED/720/point_cloud_PLY_3029_720_06-12-2022-19-39-57.ply'

# %%
with open(LiDAR_data, newline='') as f:
    rows = list(csv.reader(f, delimiter=',', quotechar='"'))
    sph_lidar = np.zeros(shape=(len(rows) - 1, 3))
    headers = rows.pop(0)
    for index, row in enumerate(rows[1:]):
        # print(index, row[0])
        sph_lidar[index] = row[7:10]

gt_mean = sph_lidar[:,2].mean()

# %%
indexer = {
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

sph_lidar[:,0] = np.array(list(map(indexer.get, sph_lidar[:,0])))

# %%
mid_cutoff = 18000
sph_lidar[:,1] += mid_cutoff

mask = sph_lidar[:,1] > 2*mid_cutoff

sph_lidar[mask, 1] -= 2*mid_cutoff

# %%
num_arrays = 16

# Split the array based on the range of the first column
arrays = []
for i in range(num_arrays):
    lower = i
    upper = i + 1
    mask = (sph_lidar[:, 0] >= lower) & (sph_lidar[:, 0] < upper)
    sub_array = sph_lidar[mask, 1:]
    arrays.append(sub_array)

# Convert each sub-array to a 1D array sorted by the first column and using the second column as the value
sph_lidar_frame = []
for sub_array in arrays:
    indices = np.argsort(sub_array[:, 0])
    sorted_array = sub_array[indices, 1].tolist()

    row_len = len(sorted_array)
    num_zeros = 1280 - row_len
    step_size = row_len // (num_zeros + 1)

    # Loop over the array and insert zeros at regular intervals
    for i in range(num_zeros):
        index = (i + 1) * step_size
        sorted_array.insert(index, gt_mean)
    print(len(sorted_array))
    
    sph_lidar_frame.append(sorted_array)

# %%
def appendSpherical_np(xyz):
    ptsnew = np.hstack((xyz, np.zeros(xyz.shape)))
    xy = xyz[:,2]**2 + xyz[:,1]**2
    ptsnew[:,3] = np.sqrt(xy + xyz[:,0]**2)
    # ptsnew[:,4] = np.arctan2(np.sqrt(xy), xyz[:,0]) # for elevation angle defined from Z-axis down
    ptsnew[:,4] = np.arctan2(xyz[:,0], np.sqrt(xy)) # for elevation angle defined from XY-plane up
    ptsnew[:,5] = np.arctan2(xyz[:,1], xyz[:,2])
    return ptsnew

# %%
pcd_zed = o3d.io.read_point_cloud(ZED_data)
pts_zed = np.asarray(pcd_zed.points)

# %%
pts_zed_sph = appendSpherical_np(pts_zed)[:,3:6]
pts_zed_sph[:,1:3] = np.degrees(pts_zed_sph[:,1:3]) + 180

# %%
pts_zed_sph[:,2].shape

# %%
a = pts_zed_sph[:,2]

lower_angle = 20
upper_angle = 20

polar_min, polar_max = pts_zed_sph[:,2].min() + lower_angle, pts_zed_sph[:,2].max() - upper_angle

pts_zed_sph_filt = pts_zed_sph[(a < polar_min) | (a > polar_max)]
# pts_zed_sph_filt = b[b[:,2] < max]

print(pts_zed_sph_filt.shape)

# %%
pts_zed_sph_filt[:,1] *= 14.03
pts_zed_sph_filt[:,1] = pts_zed_sph_filt[:,1]//1 - 1903
print(len(np.unique(pts_zed_sph_filt[:,1])))

h_range = (int(pts_zed_sph_filt[:,1].min()), int(pts_zed_sph_filt[:,1].max()+1))

# %%
bg_depth = pts_zed_sph_filt[:,0].mean()
print(bg_depth)

# %%
def map_range_to_interval(value, old_min, old_max, new_min, new_max):
    mapped_value = (value - old_min) * (new_max - new_min) / (old_max - old_min) + new_min
    return int(mapped_value)

# %%
mask = pts_zed_sph_filt[:,2] > lower_angle

pts_zed_sph_filt[mask, 2] -= 360
pts_zed_sph_filt[not mask.all(), 2] += lower_angle

pts_zed_sph_filt[:,2] *= 1.6

v_range = (pts_zed_sph_filt[:,2].min(), pts_zed_sph_filt[:,2].max()+1)

pts_zed_sph_filt[:,2] = np.vectorize(map_range_to_interval)(pts_zed_sph_filt[:,2], v_range[0], v_range[1], 0, 720)

# %%
sph_zed_frame = []
for i in range(0, 720):
    lower = i
    upper = i + 1
    mask = (pts_zed_sph_filt[:, 2] >= lower) & (pts_zed_sph_filt[:, 2] < upper)
    sub_array = pts_zed_sph_filt[mask, 0:2]

    row = np.ones(1280)*bg_depth
    row[sub_array[:,1].astype(int)] = sub_array[:,0]
    sph_zed_frame.append(row)
sph_zed_frame = np.asarray(sph_zed_frame)

# %%
m = 720//16

# us = np.ones((720,1280)) * gt_mean /2
us = sph_zed_frame
us[::m,:] = sph_lidar_frame

plt.imshow(us)

# %%
def lpf(img, ncutoff):
    # Apply 2D FFT to the image
    f = cp.fft.fft2(img)

    # Shift the zero frequency component to the center of the spectrum
    fshift = cp.fft.fftshift(f)

    # Create a circular mask of the same size as the spectrum
    rows, cols = img.shape
    crow, ccol = rows // 2, cols // 2
    mask = np.zeros((rows, cols), np.uint8)
    cutoff = int(min(crow, ccol)*ncutoff)
    cv2.circle(mask, (ccol, crow), cutoff, 1, -1)
    # cv2.ellipse(mask, (ccol, crow), (1, 2) * cutoff, 0, 0, 360,  1, -1)

    mask = cp.asarray(mask)

    # Apply the mask to the shifted spectrum
    fshift_filtered = fshift * mask

    # Shift the zero frequency component back to the corner of the spectrum
    f_filtered = cp.fft.ifftshift(fshift_filtered)

    # Apply the inverse 2D FFT to the filtered spectrum
    img_filtered = cp.fft.ifft2(f_filtered)
    img_filtered = cp.real(img_filtered)

    return img_filtered


def pg(input, us_rate, ncutoff, threshold = 100):
    ncutoff = ncutoff / 10
    filtered = input
    
    while threshold > 0:
        filtered = lpf(filtered,ncutoff)
        filtered[::us_rate, ::us_rate] = input[::us_rate, ::us_rate]

        threshold -=1
    
    return filtered

# %%
plt.imshow(cp.log(abs(cp.fft.fftshift(cp.fft.fft2(cp.asarray(us))))).get())

# %%
img_l = lpf(cp.asarray(us), 0.25)
plt.imshow(cp.log(abs(cp.fft.fftshift(cp.fft.fft2(img_l)))).get())
img_l

# %%
img_l[::m,:] = cp.asarray(us)[::m,:]
plt.imshow(cp.log(abs(cp.fft.fftshift(cp.fft.fft2(cp.asarray(img_l))))).get())

# %%
pg_frame = pg(cp.asarray(us), m, ncutoff=1, threshold=1)
plt.imshow(pg_frame.get())

# %%
pg_frame - us

# %%
print(us.max(), pg_frame.max())

# %%
def back_to_pts_form(arr):
    # get the shape of the input array
    m, n = arr.shape
    azimuth_const = 100/n
    polar_const = 30/m
    
    # create a 3D output array of size (m * n, 3)
    out = np.zeros((m * n, 3))
    
    # populate the output array
    for row in range(m):
        for col in range(n):
            index = row * n + col
            out[index, 0] = arr[row, col]
            out[index, 1] = row * polar_const
            out[index, 2] = col * azimuth_const 
    
    return out


# %%
angle = {
    15:-15,
    14:-13,
    13:-11,
    12:-9,
    11:-7,
    10:-5,
    9:-3,
    8:-1,
    7:1,
    6:3,
    5:5,
    4:7,
    3:9,
    2:11,
    1:13,
    0:15,
}

# %%
back_pts = back_to_pts_form(pg_frame)
# back_pts[:,1] = np.array(list(map(angle.get, back_pts[:,1])))
back_pts[:,1] = np.radians(back_pts[:,1])
back_pts[:,2] = np.radians(back_pts[:,2])

# %%
max(back_pts[:,2])

# %%
point_cloud_data = back_pts

# Convert spherical coordinates to Cartesian coordinates
x = point_cloud_data[:, 0] * np.cos(point_cloud_data[:, 1]) * np.cos(point_cloud_data[:, 2])
y = point_cloud_data[:, 0] * np.cos(point_cloud_data[:, 1]) * np.sin(point_cloud_data[:, 2])
z = point_cloud_data[:, 0] * np.sin(point_cloud_data[:, 1])

pg_pts = np.asarray([x, y, z]).T

pcd_pg_lidar = o3d.geometry.PointCloud()
pcd_pg_lidar.points = o3d.utility.Vector3dVector(pg_pts)

# %%
pg_pts.shape

# %%
sph_lidar[:,0] = np.radians(sph_lidar[:,0])
sph_lidar[:,1] = np.radians(sph_lidar[:,1] / 100)

point_cloud_data = sph_lidar

# Convert spherical coordinates to Cartesian coordinates
x = point_cloud_data[:, 2] * np.cos(point_cloud_data[:, 0]) * np.cos(point_cloud_data[:, 1])
y = point_cloud_data[:, 2] * np.cos(point_cloud_data[:, 0]) * np.sin(point_cloud_data[:, 1])
z = point_cloud_data[:, 2] * np.sin(point_cloud_data[:, 0])

inp_pts = np.asarray([x, y, z]).T

pcd_inp_lidar = o3d.geometry.PointCloud()
pcd_inp_lidar.points = o3d.utility.Vector3dVector(inp_pts)

# %%

o3d.visualization.draw_geometries([
    pcd_pg_lidar, 
    # pcd_inp_lidar
    ])



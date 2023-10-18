import rospy
from sensor_msgs.msg import PointCloud2, Image, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from velodyne_msgs.msg import VelodyneScan
import numpy as np
import cupy as cp
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import time
import cv2
import open3d as o3d
import math

ZED_TOPIC = "/islam/zed_pts"
VLP_TOPIC = "/islam/vlp_pts"
ZED_DEPTH = '/zed/zed_node/depth/depth_registered'
ZED_V = 360
ZED_H = 640
ZED_H_ANGLE = 90
ZED_V_ANGLE = 60

LiDAR_V = 16
LiDAR_ANGLE = 32  # 2 degree precaution

row_theta_step = 2
row_theta_map = ZED_V/(2 * LiDAR_V)

col_phi_map = ZED_H/ZED_H_ANGLE
col_phi_max = ZED_H_ANGLE/2

m = ZED_V//(2 * LiDAR_V)
def sph_to_cart_pts(pts):
    pts[:,1] = cp.radians(pts[:,1])
    pts[:,2] = cp.radians(pts[:,2])

    # Convert spherical coordinates to Cartesian coordinates
    x = pts[:, 0] * cp.cos(pts[:, 1]) * cp.cos(pts[:, 2])
    y = pts[:, 0] * cp.cos(pts[:, 1]) * cp.sin(pts[:, 2])
    z = pts[:, 0] * cp.sin(pts[:, 1])
    
    return cp.asarray([x, y, z]).T

def cart_to_sph_pts(pts):
    # Convert to CuPy array
    pts = cp.asarray(pts)

    # Convert to spherical coordinates
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = cp.sqrt(x**2 + y**2 + z**2)
    theta = cp.arctan(z / cp.sqrt(x**2 + y**2))
    phi = cp.arctan(y / x)

    return cp.column_stack((r, cp.degrees(theta), cp.degrees(phi)))
    # return appendSpherical_np(pts)[:,3:6]

def msg2pts(msg):
    return cp.array(list(pc2.read_points(msg, field_names=("x", "y", "z"))))
def depth_to_sph_pts(depth):
    # Make sure input depth array is a CuPy array
    depth = cp.array(depth)
    
    # get the shape of the input array
    m, n = depth.shape
    azimuth_const = ZED_H_ANGLE/n
    polar_const = ZED_V_ANGLE/m

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


def pg(zed_depth, vlp_depth, ncutoff, threshold=100):
    ncutoff = ncutoff / 10
    
    mask = vlp_depth > 0
    filtered = zed_depth
    filtered[mask] = vlp_depth[mask]
    
    while threshold > 0:
        filtered = lpf(filtered, ncutoff)
        filtered[mask] = vlp_depth[mask]

        threshold -= 1

    return filtered
rospy.init_node('sf', anonymous=True)
pg_pts_p = rospy.Publisher("/islam/pg_pts", PointCloud2, queue_size=50)

while True:
    zed_img = rospy.wait_for_message(ZED_DEPTH, Image)
    vlp_pts = rospy.wait_for_message(VLP_TOPIC, PointCloud2)
    if zed_img and vlp_pts:
        # VLP Preproc
        vlp_pts = msg2pts(vlp_pts)
        vlp_sph_pts_raw = cart_to_sph_pts(vlp_pts[vlp_pts[:,0] > 0])
        mask = (vlp_sph_pts_raw[:, 2] < ZED_H_ANGLE/2) & (vlp_sph_pts_raw[:, 2] > -ZED_H_ANGLE/2)
        vlp_sph_pts = vlp_sph_pts_raw[mask]
        
        vlp_depth = cp.zeros(shape=(ZED_V, ZED_H))

        theta_max = vlp_sph_pts[:,1].max()
        theta_min = vlp_sph_pts[:,1].min()
        theta_range = theta_max - theta_min

        for i in range(0, LiDAR_V):
            mask = (vlp_sph_pts[:, 1] < theta_max - i*theta_range/LiDAR_V) & (vlp_sph_pts[:, 1] > theta_max - (i+1)*theta_range/LiDAR_V)
            row = vlp_sph_pts[mask]
            
            cols = []
            for col in row:
                px = math.floor(ZED_H - float(col[2] + ZED_H_ANGLE/2) * ZED_H / ZED_H_ANGLE)
                cols.append(px)
                vlp_depth[int(ZED_V/4 + i * row_theta_map), px - 1]= col[0]
                
        # ZED Preproc
        bridge = CvBridge()
        zed_depth = cp.array(bridge.imgmsg_to_cv2(zed_img, "32FC1"))
        zed_depth[cp.isnan(zed_depth)] = cp.mean(vlp_sph_pts[:,0])
        zed_depth[zed_depth > 20] = cp.mean(vlp_sph_pts[:,0])
        
        # Sensor Fusion
        pg_depth = pg(zed_depth.copy(), vlp_depth.copy(), ncutoff=3, threshold=50)
        
        # Publish PC2
        pg_pts = sph_to_cart_pts(depth_to_sph_pts(pg_depth))

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        
        msg_p = pc2.create_cloud_xyz32(header, pg_pts.get()) 

        pg_pts_p.publish(msg_p)
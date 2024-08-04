#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

def convert_depth_image_to_pointcloud(depth_image):
    # Assuming depth_image is a numpy array with dtype=np.uint16
    # Scale the depth image to get the depth in meters
    depth_in_meters = depth_image.astype(np.float32) / 256.0

    # Create a mask to filter out invalid (zero) depth pixels
    valid_mask = depth_in_meters > 0

    # Calculate x, y, z coordinates from the depth image and intrinsic parameters
    height, width = depth_image.shape
    x, y = np.meshgrid(np.arange(width), np.arange(height))
    
    # Placeholder values for intrinsic parameters, replace these with your actual values
    fx, fy = 1, 1  # Focal lengths in pixels
    cx, cy = width // 2, height // 2  # Optical center in pixels
    
    # Unproject
    x = (x - cx) * depth_in_meters / fx
    y = (y - cy) * depth_in_meters / fy
    z = depth_in_meters

    # Stack and reshape to point cloud format
    points_3d = np.dstack((x, y, z)).reshape(-1, 3)

    # Filter out invalid points
    valid_points = points_3d[valid_mask.ravel()]

    return valid_points

def create_point_cloud2_msg(points, frame_id):
    header = rospy.Header(frame_id=frame_id)
    header.stamp = rospy.Time.now()
    fields = [pc2.PointField(name=n, offset=i*4, datatype=pc2.PointField.FLOAT32, count=1)
              for i, n in enumerate('xyz')]
    return pc2.create_cloud(header, fields, points)

def kitti_publisher():
    rospy.init_node('kitti_publisher', anonymous=True)
    lidar_pub = rospy.Publisher("/kitti/lidar", PointCloud2, queue_size=2)
    image_pub = rospy.Publisher("/kitti/rgb", Image, queue_size=2)
    bridge = CvBridge()

    lidar_path = "..data/val_se;ec"  # Update this path
    image_path = "..data/image"  # Update this path

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        for frame_id in range(10):  # Replace NUM_FRAMES with the number of frames in your dataset
            lidar_file = f"{lidar_path}/frame_{frame_id:06d}.png"
            image_file = f"{image_path}/frame_{frame_id:06d}.png"

            # Read LiDAR depth image
            depth_image = cv2.imread(lidar_file, cv2.IMREAD_UNCHANGED)
            points_3d = convert_depth_image_to_pointcloud(depth_image)

            # Publish LiDAR as PointCloud2
            pointcloud_msg = create_point_cloud2_msg(points_3d, "lidar_frame")
            lidar_pub.publish(pointcloud_msg)

            # Publish RGB Image
            cv_image = cv2.imread(image_file)
            image_message = bridge.cv2_to_imgmsg(cv_image, "bgr8")
            image_pub.publish(image_message)

            rate.sleep()

if __name__ == '__main__':
    try:
        kitti_publisher()
    except rospy.ROSInterruptException:
        pass

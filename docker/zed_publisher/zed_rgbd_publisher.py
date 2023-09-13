import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import pyzed.sl as sl

def main():
    # Initialize ROS
    rospy.init_node('zed_rgbd_publisher')
    bridge = CvBridge()
    rgbd_pub = rospy.Publisher('/zed/rgbd_image', Image, queue_size=1)
    camera_info_pub = rospy.Publisher('/zed/rgbd/camera_info', CameraInfo, queue_size=1)

    # Create a ZED camera object
    zed = sl.Camera()

    # Set up camera configuration
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.camera_fps = 30
    

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        rospy.logerr('Failed to open ZED camera')
        sys.exit(1)

    # Retrieve camera information
    camera_info = zed.get_camera_information()
    camera_info_msg = CameraInfo()
    # Fill in camera_info_msg with relevant information from camera_info

    # Main loop
    rate = rospy.Rate(30) # Adjust the rate based on your camera FPS
    bridge = CvBridge()
    while not rospy.is_shutdown():
        # Grab a new frame
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            rospy.logerr('Failed to grab a new frame from ZED camera')
            break

        # Retrieve the depth map
        depth_map = sl.Mat()
        zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)

        # Retrieve the RGB image
        rgb_image = sl.Mat()
        zed.retrieve_image(rgb_image, sl.VIEW.LEFT)

        # Convert the depth map and RGB image to numpy arrays
        depth_image = depth_map.get_data()
        depth_image = np.nan_to_num(depth_image)
        rgb_image = rgb_image.get_data()

        # Create a combined RGBD image
        rgbd_image = np.concatenate((rgb_image, depth_image[..., np.newaxis]), axis=2)

        # Create a ROS sensor_msgs.Image message
        rgbd_msg = bridge.cv2_to_imgmsg(rgbd_image, encoding='rgb8')

        # Publish the RGBD image
        rgbd_msg.header.stamp = rospy.Time.now()
        rgbd_pub.publish(rgbd_msg)

        # Publish the camera info
        camera_info_msg.header = rgbd_msg.header
        camera_info_pub.publish(camera_info_msg)

        rate.sleep()

    # Close the camera
    zed.close()

if __name__ == '__main__':
    main()

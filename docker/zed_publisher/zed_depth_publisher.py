import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import pyzed.sl as sl

def main():
    # Initialize ROS
    rospy.init_node('zed_depth_publisher')
    bridge = CvBridge()
    depth_pub = rospy.Publisher('/islam/zed/depth', Image, queue_size=1)
    rgb_pub = rospy.Publisher('/islam/zed/rgb', Image, queue_size=1)
    camera_info_pub = rospy.Publisher('/islam/zed/camera_info', CameraInfo, queue_size=1)

    # Create a ZED camera object
    zed = sl.Camera()

    # Set up camera configuration
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.camera_fps = 30
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_minimum_distance = 0.15 # Set the minimum depth perception distance to 15cmv

    # Set sensing mode in FILL
    runtime_parameters =sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL
    
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
        depth_map = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.F32_C1)
        zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)

        # Convert the depth map to a numpy array
        depth_image = depth_map.get_data()

        # Create a ROS sensor_msgs.Image message
        depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')

        # Publish the depth image
        depth_msg.header.stamp = rospy.Time.now()
        depth_pub.publish(depth_msg)

        # Publish the camera info
        camera_info_msg.header = depth_msg.header
        camera_info_pub.publish(camera_info_msg)
        
        # Retrieve the RGB image
        rgb_image = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.F32_C3)
        zed.retrieve_image(rgb_image, sl.VIEW.LEFT)

        # Convert the depth map and RGB image to numpy arrays
        rgb_image = rgb_image.get_data()
        
        # Convert the image from 4 channel (RGBA) to 3 channel (RGB)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGBA2RGB)

        # Create a ROS sensor_msgs.Image message
        rgb_msg = bridge.cv2_to_imgmsg(rgb_image, encoding='32FC3')

        # Publish the RGB image
        rgb_msg.header.stamp = rospy.Time.now()
        rgb_pub.publish(rgb_msg)

        rate.sleep()

    # Close the camera
    zed.close()

if __name__ == '__main__':
    main()

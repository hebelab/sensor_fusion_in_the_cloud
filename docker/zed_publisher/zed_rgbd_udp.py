import socket
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import pickle
import struct

def main():
    # Initialize ROS
    rospy.init_node('zed_rgbd_publisher')
    bridge = CvBridge()
    rgbd_pub = rospy.Publisher('/zed/rgbd_image', Image, queue_size=1)

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind the socket to a specific network interface and port number
    server_address = ('192.168.123.11', 12345)
    sock.bind(server_address)

    while not rospy.is_shutdown():
        # Receive data
        data, address = sock.recvfrom(65507) # Max UDP Packet size for IPv4

        # Deserialize the data
        rgbd_image = pickle.loads(data)

        # Split RGB-D image into separate RGB and Depth images
        rgb_image = rgbd_image[..., :3]
        depth_image = rgbd_image[..., 3]

        # Convert the RGB image to a ROS sensor_msgs.Image message
        rgb_msg = bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
        rgb_msg.header.stamp = rospy.Time.now()

        # Convert the depth image to a ROS sensor_msgs.Image message
        depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding='mono16')
        depth_msg.header.stamp = rgb_msg.header.stamp

        # Publish the RGB-D images
        rgbd_pub.publish(rgb_msg, depth_msg)

    # Close the socket
    sock.close()

if __name__ == '__main__':
    main()

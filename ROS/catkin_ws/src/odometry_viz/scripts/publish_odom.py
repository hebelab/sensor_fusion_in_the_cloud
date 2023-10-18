#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose
import copy
import tf
import math

"""
rosrun tf2_ros static_transform_publisher -35 -65 40 0.0 0.0 0.0 1.0 map loam_link
rosrun tf2_ros static_transform_publisher 0 0 0 0.3535534 0.3535534 0.1464466 0.8535534 loam_link rot
"""


def transformer(odom, r=0, p=0, y=0):
    # Handle orientation
    quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    )
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    # Apply rotation
    roll += r
    pitch += p
    yaw += y
    # Convert back to quaternion
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    # Assign back to the odom message
    odom.pose.pose.orientation.x = quaternion[0]
    odom.pose.pose.orientation.y = quaternion[1]
    odom.pose.pose.orientation.z = quaternion[2]
    odom.pose.pose.orientation.w = quaternion[3]
    return odom


class OdometryPlotter:
    def __init__(self):
        self.sdk_path = Path()
        self.loam_path = Path()
        self.sdk_odom = Odometry()
        self.loam_odom = Odometry()
        
        rospy.init_node('odometry_publisher', anonymous=True)
        self.sdk_odom_subscriber = rospy.Subscriber('/sdk_odom', Odometry, self.sdk_odom_callback)
        self.sdk_odom_publisher = rospy.Publisher('/islam/ut_odom', Odometry, queue_size=10)
        self.sdk_path_publisher = rospy.Publisher('/islam/ut_path', Path, queue_size=10)
        
        self.loam_odom_subscriber = rospy.Subscriber('/aft_mapped_to_init', Odometry, self.loam_odom_callback)
        self.loam_odom_publisher = rospy.Publisher('/islam/loam_odom', Odometry, queue_size=10)
        self.loam_path_publisher = rospy.Publisher('/islam/loam_path', Path, queue_size=10)
        
        self.zed_odom_subscriber = rospy.Subscriber('/zed/zed_node/odom', Odometry, self.zed_odom_callback)
        self.loam_odom_tf_publisher = rospy.Publisher('/islam/loam_odom_tf', Odometry, queue_size=10)

        rospy.spin()
    
        
    def zed_odom_callback(self, msg):        
        self.loam_odom.header = msg.header
        self.sdk_odom.header = msg.header
        
        # msg_tf = copy.deepcopy(self.loam_odom)
        # msg_tf.header = msg.header
        
        # msg_tf = transformer(msg_tf, p=-math.pi/2)

        # self.publish_odom(msg_tf, self.loam_odom_tf_publisher, frame_id='odom')
        self.publish_odom(self.sdk_odom , self.sdk_odom_publisher, frame_id='odom', child_frame_id='base_link')
        self.publish_path(self.sdk_odom , self.sdk_path_publisher, self.sdk_path)
        self.publish_odom(self.loam_odom , self.loam_odom_publisher, frame_id='odom')
        self.publish_path(self.loam_odom, self.loam_path_publisher, self.loam_path, frame_id='rot')
        
    def sdk_odom_callback(self, msg: Odometry):
        self.sdk_odom = copy.deepcopy(msg)
        self.sdk_odom.pose.pose.orientation.x = copy.deepcopy(msg.pose.pose.orientation.z)
        self.sdk_odom.pose.pose.orientation.z = copy.deepcopy(msg.pose.pose.orientation.x)
        self.sdk_odom = transformer(self.sdk_odom, r=math.pi, p=0, y=0)
        
    def loam_odom_callback(self, msg):
        self.loam_odom = transformer(copy.deepcopy(msg), r=math.pi/2, p=-math.pi/2)
        
    def publish_path(self, msg, publisher, path, frame_id="map"):
        # Publish Path message
        
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = frame_id
        
        pose_stamped = PoseStamped()
        pose_stamped.pose = Pose(msg.pose.pose.position, msg.pose.pose.orientation)
        pose_stamped.header = path.header
        path.poses.append(pose_stamped)
        
        publisher.publish(path)

    def publish_odom(self, msg, publisher, frame_id="map", child_frame_id='base_link'):
        # Customize the Odometry message
        odom = msg
        # odom.header.stamp = rospy.Time.now()
        # odom.header.frame_id = frame_id
        # odom.child_frame_id = child_frame_id

        # Publish the Odometry message
        publisher.publish(odom)        

if __name__ == '__main__':
    try:
        OdometryPlotter()
    except rospy.ROSInterruptException:
        pass

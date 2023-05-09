#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose


class OdometryPlotter:
    def __init__(self):
        rospy.init_node('odometry_publisher', anonymous=True)
        self.odom_subscriber = rospy.Subscriber('/base_odometry/odom', Odometry, self.odom_callback)
        self.odom_publisher = rospy.Publisher('odom', Odometry, queue_size=10)
        self.path_publisher = rospy.Publisher('path', Path, queue_size=10)
        
        self.path = Path()
        
        rospy.spin()
        
    def odom_callback(self, msg):
        self.publish_odom(msg)
        self.publish_path(msg)
        
    def publish_path(self, msg):
        # Publish Path message
        
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "odom"
        pose_stamped = PoseStamped()
        pose_stamped.pose = Pose(msg.pose.pose.position, msg.pose.pose.orientation)
        pose_stamped.header = self.path.header
        self.path.poses.append(pose_stamped)
        
        self.path_publisher.publish(self.path)

    def publish_odom(self, msg):
        # Create an Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set position information (example data)
        odom.pose.pose.position.x = msg.pose.pose.position.x
        odom.pose.pose.position.y = msg.pose.pose.position.y
        odom.pose.pose.position.z = msg.pose.pose.position.z

        # Set orientation information (example data)
        # roll, pitch, yaw = 0.0, 0.0, 0  # In radians
        # quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        # odom.pose.pose.orientation = Quaternion(*quaternion)
        odom.pose.pose.orientation = msg.pose.pose.orientation

        # Set linear and angular velocity information (example data)
        odom.twist.twist.linear.x = msg.twist.twist.linear.x
        odom.twist.twist.linear.y = msg.twist.twist.linear.y
        odom.twist.twist.linear.z = msg.twist.twist.linear.z
        odom.twist.twist.angular.x = msg.twist.twist.angular.x
        odom.twist.twist.angular.y = msg.twist.twist.angular.y
        odom.twist.twist.angular.z = msg.twist.twist.angular.z

        # Publish the Odometry message
        self.odom_publisher.publish(odom)        

if __name__ == '__main__':
    try:
        OdometryPlotter()
    except rospy.ROSInterruptException:
        pass

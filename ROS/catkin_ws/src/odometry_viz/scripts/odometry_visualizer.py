#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import math

class OdometryVisualizer:
    def __init__(self):
        rospy.init_node('odometry_visualizer', anonymous=True)
        self.odom_subscriber = rospy.Subscriber('/integrated_to_init', Odometry, self.odom_callback)
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.spin()

    def odom_callback(self, msg):
        # Extract pose and orientation from the Odometry message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform((position.x, position.y, position.z),
                                          (orientation.x, orientation.y, orientation.z, orientation.w),
                                          rospy.Time.now(),
                                          'odom_visualization',
                                          'odom')

if __name__ == '__main__':
    try:
        OdometryVisualizer()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, Twist, PoseWithCovariance, TwistWithCovariance
from tf.transformations import quaternion_matrix, quaternion_from_matrix


class OdometryPlotter:
    def __init__(self):
        rospy.init_node('odometry_publisher', anonymous=True)
        self.zed_subscriber = rospy.Subscriber('/zed/zed_node/odom', Odometry, self.zed_callback)
        self.loam_subscriber = rospy.Subscriber('/aft_mapped_to_init', Odometry, self.loam_callback)
        self.odom_publisher = rospy.Publisher('/islam/odom', Odometry, queue_size=10)
        self.transformation_matrix = np.array([[0.93474884, 0.35143906, 0.05229909, 21.37571084],
                                               [-0.16145324, 0.28900616, 0.94361448, 30.36377992],
                                               [0.31650823, -0.8904864, 0.32688915, -39.69413224],
                                               [0., 0., 0., 1.]])
        
        self.odom = Odometry()

        
        rospy.spin()

    def transform_odometry(self, odom):
        # Convert quaternion to rotation matrix
        quaternion = [odom.pose.pose.orientation.x,
                      odom.pose.pose.orientation.y,
                      odom.pose.pose.orientation.z,
                      odom.pose.pose.orientation.w]
        rotation_matrix = quaternion_matrix(quaternion)

        # Apply transformation
        transformed_rotation_matrix = np.dot(self.transformation_matrix, rotation_matrix)

        # Convert back to quaternion
        transformed_quaternion = quaternion_from_matrix(transformed_rotation_matrix)

        # Apply transformation to position
        position = np.array([odom.pose.pose.position.x,
                             odom.pose.pose.position.y,
                             odom.pose.pose.position.z,
                             1])
        transformed_position = np.dot(self.transformation_matrix, position)

        # Apply transformed position and orientation
        odom.pose.pose.position.x = transformed_position[0]
        odom.pose.pose.position.y = transformed_position[1]
        odom.pose.pose.position.z = transformed_position[2]
        odom.pose.pose.orientation.x = transformed_quaternion[0]
        odom.pose.pose.orientation.y = transformed_quaternion[1]
        odom.pose.pose.orientation.z = transformed_quaternion[2]
        odom.pose.pose.orientation.w = transformed_quaternion[3]
        return odom

    def zed_callback(self, msg):
        self.odom.header = msg.header
        self.odom.child_frame_id = msg.child_frame_id
        self.odom_publisher.publish(self.odom)

    def loam_callback(self, msg):
        transformed_odom = self.transform_odometry(msg)
        self.odom.pose = transformed_odom.pose
        self.odom.twist = transformed_odom.twist
        self.odom_publisher.publish(self.odom)


if __name__ == '__main__':
    try:
        OdometryPlotter()
    except rospy.ROSInterruptException:
        pass

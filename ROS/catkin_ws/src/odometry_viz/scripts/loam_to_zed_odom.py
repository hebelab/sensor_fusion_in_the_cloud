#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, Twist


class OdometryPlotter:
    def __init__(self):
        
        rospy.init_node('odometry_publisher', anonymous=True)
        self.zed_subscriber = rospy.Subscriber('/zed/zed_node/odom', Odometry, self.zed_callback)
        self.loam_subscriber = rospy.Subscriber('/aft_mapped_to_init', Odometry, self.loam_callback)
        self.odom_publisher = rospy.Publisher('/islam/odom', Odometry, queue_size=10)
        
        self.path = Path()
        self.odom = Odometry()

        rospy.spin()
        
    def zed_callback(self, msg):
        self.odom.header = msg.header
        self.odom.header.frame_id = 'map'
        self.odom.child_frame_id = msg.child_frame_id
        self.odom_publisher.publish(self.odom)
        
    def loam_callback(self, msg):
        self.odom.pose = msg.pose
        self.odom.twist = msg.twist



if __name__ == '__main__':
    try:
        OdometryPlotter()
    except rospy.ROSInterruptException:
        pass
    
"""
roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed/zed_node/rgb/image_rect_color \
    depth_topic:=/zed/zed_node/depth/depth_registered \
    camera_info_topic:=/zed/zed_node/depth/camera_info \
    odom_topic:=/islam/loam_odom \
    visual_odometry:=false \
    frame_id:=base_link \
    approx_sync:=false \
    rgbd_sync:=true \
    approx_rgbd_sync:=false
    
    
roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed/zed_node/rgb/image_rect_color \
    depth_topic:=/islam/pg_depth \
    camera_info_topic:=/zed/zed_node/depth/camera_info \
    odom_topic:=/islam/loam_odom \
    visual_odometry:=false \
    frame_id:=base_link \
    approx_sync:=false \
    rgbd_sync:=true \
    approx_rgbd_sync:=false
"""
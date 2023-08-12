#!/bin/sh

frame_id="map"  # common frame_id
topic_base="islam"

export zed_pts_s="/zed/zed_node/point_cloud/cloud_registered"
export zed_pts_p="/"$topic_base"/zed_pts"

export vlp_pts_s="/velodyne_points"
export vlp_pts_p="/"$topic_base"/vlp_pts"

export sdk_imu_s="/sdk_imu"
export sdk_imu_p="/"$topic_base"/ut_imu"

export sdk_odom_s="/sdk_odom"
export sdk_odom_p="/"$topic_base"/ut_odom"

export loam_map_s="/laser_cloud_surround"
export loam_map_p="/"$topic_base"/loam_map"

# rosrun topic_tools transform $zed_pts_s $zed_pts_p sensor_msgs/PointCloud2 'sensor_msgs.msg.PointCloud2(header=std_msgs.msg.Header(seq=m.header.seq,stamp=m.header.stamp,frame_id="'$frame_id'"), data=m.data, height=m.height, width=m.width, fields=m.fields, is_bigendian=m.is_bigendian, point_step=m.point_step, row_step=m.row_step, is_dense=m.is_dense)' --import std_msgs sensor_msgs&

rosrun topic_tools transform $vlp_pts_s $vlp_pts_p sensor_msgs/PointCloud2 'sensor_msgs.msg.PointCloud2(header=std_msgs.msg.Header(seq=m.header.seq,stamp=m.header.stamp,frame_id="'$frame_id'"), data=m.data, height=m.height, width=m.width, fields=m.fields, is_bigendian=m.is_bigendian, point_step=m.point_step, row_step=m.row_step, is_dense=m.is_dense)' --import std_msgs sensor_msgs&

# rosrun topic_tools transform $sdk_imu_s $sdk_imu_p sensor_msgs/Imu 'sensor_msgs.msg.Imu(header=std_msgs.msg.Header(seq=m.header.seq,stamp=m.header.stamp,frame_id="'$frame_id'"), orientation=m.orientation, orientation_covariance=m.orientation_covariance, angular_velocity=m.angular_velocity, angular_velocity_covariance=m.angular_velocity_covariance, linear_acceleration=m.linear_acceleration, linear_acceleration_covariance=m.linear_acceleration_covariance)' --import std_msgs sensor_msgs&

# rosrun tf static_transform_publisher 0 0 0 0 0 1.5708 /camera_init /camera_init_z90 30&
rosrun topic_tools transform $loam_map_s $loam_map_p sensor_msgs/PointCloud2 'sensor_msgs.msg.PointCloud2(header=std_msgs.msg.Header(seq=m.header.seq,stamp=m.header.stamp,frame_id="'$frame_id'"), data=m.data, height=m.height, width=m.width, fields=m.fields, is_bigendian=m.is_bigendian, point_step=m.point_step, row_step=m.row_step, is_dense=m.is_dense)' --import std_msgs sensor_msgs&

# rosrun topic_tools transform $sdk_odom_s $sdk_odom_p nav_msgs/Odometry 'nav_msgs.msg.Odometry(header=std_msgs.msg.Header(seq=m.header.seq,stamp=m.header.stamp,frame_id="'$frame_id'"), child_frame_id=m.child_frame_id, pose=m.pose, twist=m.twist)' --import nav_msgs std_msgs&

rosrun tf2_ros static_transform_publisher -35 -65 40 0.0 0.0 0.0 1.0 map loam_link&
rosrun tf2_ros static_transform_publisher 0 0 0 0.3535534 0.3535534 0.1464466 0.8535534 loam_link rot&
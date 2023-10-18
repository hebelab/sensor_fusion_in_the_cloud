#!/bin/bash

source ~/ros_ws/devel/setup.bash
roslaunch velodyne_pointcloud VLP16_points.launch device_ip:="${SENSOR_SUIT_IP}" frame_id:="map" port:="${VLP_PORT}"
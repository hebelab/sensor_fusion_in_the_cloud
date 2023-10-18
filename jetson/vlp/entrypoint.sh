#!/bin/bash

source ~/ros_ws/devel/setup.bash
roslaunch velodyne_pointcloud VLP16_points.launch device_ip:="192.168.1.201" frame_id:="map" port:="2368"
#!/bin/bash

source ~/ros_ws/devel/setup.bash
roslaunch velodyne_pointcloud VLP16_points.launch device_ip:="${VLP_IP}" port:="${VLP_STREAM_PORT}" frame_id:="base_link" max_range:="30.0"
#!/bin/bash

cd ~/catkin_ws
source devel/setup.bash

roslaunch --wait zed_wrapper zed.launch stream:="${SENSOR_SUIT_IP}:${ZED_STREAM_PORT}" # Edit <zed-ros-wrapper/blob/master/zed_wrapper/params/common.yaml> to change parameters
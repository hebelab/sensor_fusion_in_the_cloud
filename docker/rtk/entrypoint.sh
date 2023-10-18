#!/bin/bash

source /opt/ros/noetic/setup.bash
cd ~/catkin_ws

# rm -rf build/ devel/
# catkin_make -DPYTHON_VERSION=3
# pip install -r src/rtk-ros-wrapper/requirements.txt

source devel/setup.bash

roslaunch rtk_wrapper RTK.launch
#!/bin/bash

cd ~/catkin_ws
source /opt/ros/indigo/setup.bash 
source devel/setup.bash
roslaunch loam_velodyne loam_velodyne.launch rviz:=false
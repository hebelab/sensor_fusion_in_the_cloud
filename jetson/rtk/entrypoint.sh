#!/bin/bash

source /opt/ros/noetic/setup.bash
cd /rtk/catkin_ws

source devel/setup.bash

roslaunch --wait rtk_wrapper RTK.launch

#python3 /rtk/catkin_ws/src/rtk_wrapper/src/client_tcp-serial.py -i 10.225.14.199 &
# python3 /rtk/catkin_ws/src/rtk_wrapper/src/client_tcp-serial.py -i 10.225.20.231 &
# python3 /rtk/catkin_ws/src/rtk_wrapper/src/RTK_ROS.py &

tail -f /dev/null

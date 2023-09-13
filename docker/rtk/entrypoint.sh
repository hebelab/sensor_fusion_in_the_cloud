#!/bin/bash

source /opt/ros/noetic/setup.bash

#python3 /rtk/catkin_ws/src/rtk_wrapper/src/client_tcp-serial.py -i 10.225.14.199 &

python3 /rtk/catkin_ws/src/rtk_wrapper/src/client_tcp-serial.py -i 10.225.20.231 &

python3 /rtk/catkin_ws/src/rtk_wrapper/src/RTK_ROS.py &

tail -f /dev/null

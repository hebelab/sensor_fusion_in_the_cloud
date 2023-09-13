# Setup

```bash
sudo apt install ros-<version_name>-velodyne
```

# Record

```bash
rosbag record \
	/zed/zed_node/depth/camera_info \
	/zed/zed_node/depth/camera_info \
	/zed/zed_node/odom \
	/zed/zed_node/parameter_descriptions \
	/zed/zed_node/parameter_updates \
	/zed/zed_node/path_map \
	/zed/zed_node/path_odom \
	/zed/zed_node/plane \
	/zed/zed_node/plane_marker \
	/zed/zed_node/point_cloud/cloud_registered \
	/zed/zed_node/pose \
	/zed/zed_node/pose_with_covariance \
	/zed/zed_node/rgb/camera_info \
	/zed/zed_node/rgb/image_rect_color \
	/clicked_point \
	/device_time \
	/diagnostics \
	/gps/fix \
	/imu/data \
	/rosout \
	/rosout_agg \
	/scaled_llh \
	/scan \
	/status \
	/tf \
	/tf_static \
	/velocity \
	/velodyne_nodelet_manager/bond \
	/velodyne_nodelet_manager_driver/parameter_descriptions \
	/velodyne_nodelet_manager_driver/parameter_updates \
	/velodyne_nodelet_manager_laserscan/parameter_descriptions \
	/velodyne_nodelet_manager_laserscan/parameter_updates \
	/velodyne_nodelet_manager_transform/parameter_descriptions \
	/velodyne_nodelet_manager_transform/parameter_updates \
	/velodyne_packets \
	/velodyne_points
```

# Running

## ROS Core

Open terminal session on host machine:

```bash
roscore
```

Now you can push "CTRL+Z" on Linux host to make process run in the background.

> Our host machine will be the ROS Master for all our nodes on containers and host.

## ROS Nodes

### "zed" Container

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git

git checkout tags/v3.8.x
git switch -c fantastic_grandpa

cd ../
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source ./devel/setup.bash
```

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/stereolabs/zed-ros-examples

git checkout tags/v3.8.x
git switch -c fantastic_grandpa

cd ../
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source ./devel/setup.bash
```


### "loam_velodyne" Container

Now jump to another terminal session

```bash 
cd docker
docker-compose up -d
docker-compose exec -it loam_velodyne bash
```

On the opened inside shell terminal:

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch loam_velodyne loam_velodyne.launch rviz:=false
```

> `rviz:=false` for stop openning the RViz inside Docker container.

## ROS RPi

Go to `~/Unitree/autostart/roscore/roscore.sh`:

Change the ROS Master from:

```bash
export ROS_MASTER_URI=http://192.168.123.161:11311
export ROS_IP=192.168.123.161
```

to, YYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPP

```bash
export ROS_MASTER_URI=http://192.168.123.161:11311
export ROS_IP=192.168.123.9
```

```bash
rosrun topic_tools foreign_relay _source_master:=http://<Rpi_IP_address>:11311 _source_topic:=/source_topic _destination_topic:=/destination_topic
rosrun topic_tools foreign_relay _source_master:=http://192.168.123.161:11311 _source_topic:=/camera1/range_visual_face _destination_topic:=/cam1
```

### sdk_odom

```bash
cd Unitree/sdk/unitree_legged_sdk/build/
./odom
```


# RPi Clock

On local:

```bash
date
```

```bash
sudo date -s "Mon Aug  12 20:14:11 UTC 2014"
```


# Analysing

```bash
rosrun tf view_frames
dot -Tpdf frames.gv -o frames.pdf
```
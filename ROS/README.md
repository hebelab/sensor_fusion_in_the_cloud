# Setup

```bash
sudo apt install ros-<version_name>-velodyne
```

### loam_velodyne

```bash
git clone https://github.com/ayusufsirin/loam_velodyne
cd loam_velodyne 
git checkout -b elastic_grandpa 3feae58fdce82f2e96bc42e70c4d30438d1c8922 
```

# Running

## Topic name/frame_id Transformer

```bash
./transformer.sh
```

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

#### .bashrc

```bash
cd ~/catkin_ws
source devel/setup.bash
```

#### rtabmap

* https://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping

```bash
roslaunch zed_wrapper zed_no_tf.launch
rosrun dynamic_reconfigure dynparam set zed_node depth_confidence 99
rosrun dynamic_reconfigure dynparam set zed_node depth_texture_conf 90
rosrun dynamic_reconfigure dynparam set zed_node depth_confidence 100
```

```bash
roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed_node/rgb/image_rect_color \
    depth_topic:=/zed_node/depth/depth_registered \
    camera_info_topic:=/zed_node/rgb/camera_info \
    frame_id:=base_link \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/zed_node/imu/data
```

ALT:
```bash
roslaunch zed_wrapper zed_no_tf.launch
rosrun dynamic_reconfigure dynparam set zed/zed_node depth_confidence 99
rosrun dynamic_reconfigure dynparam set zed/zed_node depth_texture_conf 90
rosrun dynamic_reconfigure dynparam set zed/zed_node depth_confidence 100
```

```bash
roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed/zed_node/rgb/image_rect_color \
    depth_topic:=/zed/zed_node/depth/depth_registered \
    camera_info_topic:=/zed/zed_node/rgb/camera_info \
    frame_id:=map \
    approx_sync:=false \
    wait_imu_to_init:=false \
    imu_topic:=/imu/data \
    odom_topic:=/sdk_odom \
    rtabmap_viz:=false \
    rviz:=false \
    use_sim_time:=true \
    localization:=true
```

```bash
roslaunch rtabmap_launch rtabmap.launch \
   rtabmap_args:="--delete_db_on_start --Vis/CorFlowMaxLevel 5 --Stereo/MaxDisparity 200" \
   stereo_namespace:=/zed/zed_node \
   right_image_topic:=/zed/zed_node/right/image_rect_color \
   left_image_topic:=/zed/zed_node/left/image_rect_color \
   stereo:=true \
   frame_id:=map \
   odom_topic:=/ut_odom \
#    use_odom_features:=true \
```

ACTUAL:

```bash
roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed/zed_node/rgb/image_rect_color \
    depth_topic:=/zed/zed_node/depth/depth_registered \
    camera_info_topic:=/zed/zed_node/depth/camera_info \
    odom_topic:=/zed/zed_node/odom \
    visual_odometry:=false \
    frame_id:=base_link \
    approx_sync:=false \
    rgbd_sync:=true \
    approx_rgbd_sync:=false
```

PG RTABMAP:

```bash
roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/islam/pg_rgb \
    depth_topic:=/islam/pg_depth \
    camera_info_topic:=/islam/pg_camera_info \
    odom_topic:=/islam/pg_odom \
    visual_odometry:=false \
    frame_id:=base_link \
    approx_sync:=false \
    rgbd_sync:=true \
    approx_rgbd_sync:=false
```


### "loam_velodyne" Container

#### .bashrc

```bash
cd ~/catkin_ws
source devel/setup.bash
```

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

To transform the `frame_id`:

```bash
rosrun topic_tools transform /laser_cloud_surround /3d_map sensor_msgs/PointCloud2 'sensor_msgs.msg.PointCloud2(header=std_msgs.msg.Header(seq=m.header.seq,stamp=m.header.stamp,frame_id="velodyne"), data=m.data, height=m.height, width=m.width, fields=m.fields, is_bigendian=m.is_bigendian, point_step=m.point_step, row_step=m.row_step, is_dense=m.is_dense)' --import std_msgs sensor_msgs
```


## ROS RPi

Go to `~/Unitree/autostart/roscore/roscore.sh`:

Change the ROS Master from:

```bash
export ROS_MASTER_URI=http://192.168.123.161:11311
export ROS_IP=192.168.123.161
```


```bash
export ROS_MASTER_URI=http://192.168.123.1:11311
export ROS_IP=192.168.123.11
```

to, YYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPPYYYYYYYAAAAAAAPPPPPPPPP

```bash
export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.145
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

Enable a service from SD card:

```bash
sudo ln -s /lib/systemd/system/dhcpcd.service /etc/systemd/system/dhcpcd5.service
sudo ln -s /lib/systemd/system/dhcpcd.service /etc/systemd/system/multi-user.target.wants/dhcpcd.service
```


# Analysing

```bash
rosrun tf view_frames
dot -Tpdf frames.gv -o frames.pdf
```
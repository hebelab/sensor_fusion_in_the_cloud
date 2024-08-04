# KITTI ROS Wrapper

## Setup

Run containers:

```bash
docker-compose up kitti rosmaster
```

Install deps:

```bash
cd ~/catkin_ws
```

```bash
apt update
apt install python3-pip
pip install pcl
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Mount data path:

```bash
ln -s /media/joseph/Development/SFITC/kitti-depth/test_depth_completion_anonymous/ data
```

## Run

```bash
rosrun kitti_wrapper publish_data.py
```


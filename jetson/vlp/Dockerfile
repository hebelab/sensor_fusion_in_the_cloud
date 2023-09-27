# Use official ROS Melodic as base image
FROM ros:melodic

# Setup environment
ENV HOME=/home/ros
ENV ROS_PACKAGE_PATH=$HOME/ros_ws/src:/opt/ros/melodic/share

# Install dependencies
RUN apt-get update && apt-get install -y \
    apt-transport-https \
    lsb-release \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# Add the ROS repository
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# Add the ROS keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update the sources list
RUN apt-get update

# Install ros packages
RUN apt-get install -y \
    ros-melodic-velodyne \
    ros-melodic-velodyne-pointcloud \
    ros-melodic-roslint \
    && rm -rf /var/lib/apt/lists/*

# Create ros user
RUN adduser --disabled-password --gecos '' ros

# Create ros workspace
RUN mkdir -p $HOME/ros_ws/src
RUN chown -R ros:ros $HOME/ros_ws

# Switch to ros user
USER ros

# Clone velodyne driver source code into ros workspace
WORKDIR $HOME/ros_ws/src
RUN git clone https://github.com/ros-drivers/velodyne.git

# Install any necessary dependencies for ros packages
WORKDIR $HOME/ros_ws
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro melodic -y 

# Build ros packages from source
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'

# Source ros packages from bashrc
RUN echo "source $HOME/ros_ws/devel/setup.bash" >> ~/.bashrc
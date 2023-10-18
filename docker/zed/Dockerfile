FROM stereolabs/zed:3.8-devel-jetson-jp4.6
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS Noetic
RUN sudo apt update -y
RUN sudo apt install -y curl gnupg apt-utils
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo apt update -y
# RUN sudo apt install -y ros-melodic-ros-base

# Setup environment
# COPY run.sh /run.sh
# RUN /bin/bash /run.sh

CMD ["/bin/bash"]
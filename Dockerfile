# To build this Dockerfile use command:
#   docker build -t ultralytics-ros2 .

ARG HUMBLE_DIGEST=sha256:f05e713cd384afe798581f9bbf312a249102fb37a184049b9457e9fef90d137e

FROM ros:humble@${HUMBLE_DIGEST}

SHELL ["/bin/bash", "-c"]

# install essential packages
RUN apt-get update && apt-get install -y            \
    vim                                             \
    python3-pip                                     \
    ros-humble-rviz2                                \
    software-properties-common                      \
    # temporary mesa upgrade for RViz blackscreen bug see: https://github.com/ros2/rviz/issues/948 and https://bugs.launchpad.net/ubuntu/+source/mesa/+bug/2004649
    && add-apt-repository ppa:kisak/kisak-mesa      \ 
    && apt upgrade -y                               \
    && rm -rf /var/lib/apt/lists/*

# install ROS2 packages
RUN mkdir -p ~/ros2_ws/src                                                        \
    && cd ~/ros2_ws/src                                                           \
    && git clone https://github.com/hcdiekmann/ultralytics_ros2.git               \
    && cd ~/ros2_ws                                                               \
    && source /opt/ros/humble/setup.bash                                          \
    && apt-get update                                                             \
    && rosdep update --include-eol-distros                                        \
    && rosdep install --from-paths src --ignore-src -r -y

# intall required python packages
RUN cd ~/ros2_ws/src/ultralytics_ros2   \
    && apt-get update                   \
    && pip3 install -r requirements.txt
    

# build ros package
RUN source /opt/ros/humble/setup.bash   \
    && cd ~/ros2_ws                     \
    && colcon build 

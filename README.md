# ROS 2 object detection
ROS 2 object detection and tracking using [Ultralytics](https://ultralytics.com/) AI vision framework.

## Install
- #### Clone this repository into your ROS 2 workspace `src`  directory
```bash
cd ~/ros2_ws/src 
git clone https://github.com/hcdiekmann/ultralytics_ros2.git
```
- #### Install python dependencies
```bash
pip3 install -r /ultralytics_ros2/requirements.txt
```
- #### Install ROS dependencies
```bash
cd ~/ros2_ws
rosdep init
rosdep update
```
- #### Build the package and source the underlay installation
```bash
colcon build
. install/setup.bash
```

## Run
- #### Launch the YOLOv8 node
```bash
ros2 launch ultralytics_ros2 yolov8_node
```



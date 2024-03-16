#!/usr/bin/sh
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-visual-slam \
ros-humble-isaac-ros-yolov8 ros-humble-isaac-ros-tensor-rt ros-humble-isaac-ros-dnn-image-encoder \
ros-humble-isaac-ros-detectnet ros-humble-isaac-ros-triton ros-humble-isaac-ros-dnn-image-encoder \
ros-humble-isaac-ros-apriltag
sudo apt-get autoremove -y
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
python3 -m pip install -U pynetworktables

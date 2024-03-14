sudo apt-get install -y ros-humble-isaac-ros-visual-slam \
ros-humble-isaac-ros-yolov8 ros-humble-isaac-ros-tensor-rt ros-humble-isaac-ros-dnn-image-encoder \
ros-humble-isaac-ros-detectnet ros-humble-isaac-ros-triton ros-humble-isaac-ros-dnn-image-encoder && \
cd /workspaces/isaac_ros-dev && \
colcon build --symlink-install && \
source install/setup.bash
ros2 launch comms_node run_network_tables.launch.py

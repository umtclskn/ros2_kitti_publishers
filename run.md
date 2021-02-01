

source /opt/ros/foxy/setup.bash

source ./install/setup.bash

colcon build --cmake-clean-cache

ros2 run ros2_kitti_publishers kitti_publishers
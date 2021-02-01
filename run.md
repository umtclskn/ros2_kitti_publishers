

source /opt/ros/foxy/setup.bash

colcon build --cmake-clean-cache

source ./install/setup.bash

ros2 run ros2_kitti_publishers kitti_publishers
# ROS2 Kitti Dataset Publishers

Sample ROS2 publisher application that transforms and publishes the Kitti Dataset into the ROS2 messages.
The published ROS2 messages are mainly PointCloud2, Image, Imu, and MarkerArray. 
Also, this codebase can provide information on how to broadcast messages, configure CmakeList.txt, and package configurations for your basic and ROS2 projects.
You can use dummy the Kitti dataset to generate Perception, Planning, or Controller outputs in the Autonomous Robotics lifecycle.
It was written for only learning purposes, not as a library logic.

---

<div style="text-align:center">
  <img src="Ros2_Rviz2.gif" width="75%">
</div>

---

## **My Environment**

- Ubuntu 20.04.1 LTS
- ROS2 Foxy
- pcl 1.10
- OpenCV 4.2.0
- Rviz2
- rqt

---
## __Setup__

1. Download or clone this repo under your __{ROS2_Workspace}/src__ folder. ( i.e:  /home/user/ros2_example_ws/src/ )
2. Download the Kitti dataset from http://www.cvlibs.net/datasets/kitti/raw_data.php. ( i.e: /home/user/ros2_example_ws/data/2011_09_26/2011_09_26_drive_0015_sync/ )
3. Create a folder named "data" in your Ros2 workspace.  ( i.e:  /home/user/ros2_example_ws/data/ )
4. Extract the Kitti dataset that you downloaded before and copy all of the files into your "data" folder.
5. If you want to use another dataset from the Kitti's 2011_09_26_drive_0015_sync, you have to change paths in the init_file_path function in the kitti_publishers_node.cpp class.

```cpp
void KittiPublishersNode::init_file_path()
{
    path_point_cloud_ = "data/2011_09_26/2011_09_26_drive_0015_sync/velodyne_points/data/";
    path_image_gray_left_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_00/data/";
    path_image_gray_right_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_01/data/";
    path_image_color_left_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_02/data/";
    path_image_color_right_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_03/data/";
    path_oxts_ = "data/2011_09_26/2011_09_26_drive_0015_sync/oxts/data/";
}
```

## __Dataset  Directory Structure__:

```
.ros2_example_ws
├ build
├ data
├─────├ 2011_09_26
├─────────────────├ 2011_09_26_drive_0015_sync
├─────────────────────────────────────────────├ image_00
├─────────────────────────────────────────────├ image_01
├─────────────────────────────────────────────├ image_02
├─────────────────────────────────────────────├ image_03
├─────────────────────────────────────────────├ oxts
├─────────────────────────────────────────────├ velodyne_points
├─────────────────├ calib_cam_to_cam.txt
├─────────────────├ calib_imu_to_velo.txt
├─────────────────├ calib_velo_to_cam.txt
├ install
├ log
├ src
```

### __Dataset__:
1. image_00: left rectified grayscale image sequence
2. image_01: right rectified grayscale image sequence
3. image_02: left rectified color image sequence
4. image_03: right rectified color image sequence
5. oxts: odometry
6. velodyne_points: Point Cloud 3D laser scan data



## __Project  Directory Structure__

```
.ros2_kitti_publishers
├ include
├───────├ ros2_kitti_publishers
├──────────────────────────────├ kitti_publishers_node.hpp
├──────────────────────────────├ visibility.h
├──────────────────────────────├ WGS84toCartesian.hpp
├ launch
├ src
├───────├ node
├─────────────├ kitti_publishers_node.cpp
├───────├ kitti_publishers_main.cpp
├ default.rviz
├ kitti_rqt.perspective
├ package.xml
├ CMakeLists.txt
├ Ros2_Rviz2.gif
├ run.md
```

# __Pipeline__

* ## Implementing the publishers.
```cpp
  publisher_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti/point_cloud", 10);
  publisher_image_gray_left_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/left", 10);
  publisher_image_gray_right_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/right", 10);
  publisher_image_color_left_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/left", 10);
  publisher_image_color_right_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/right", 10);
  publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("kitti/imu", 10);
  publisher_nav_sat_fix_= this->create_publisher<sensor_msgs::msg::NavSatFix>("kitti/nav_sat_fix", 10);
  publisher_marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("kitti/marker_array", 10);
```


* ##  Create a timer object in the node class. All cameras are synchronized at about 10 Hz with respect to the Velodyne

```cpp
rclcpp::TimerBase::SharedPtr timer_;

timer_ = create_wall_timer(100ms, std::bind(&KittiPublishersNode::on_timer_callback, this));

```

* ##  The main operation/pipeline was implemented inside the callback function of the timer object.

```cpp
void KittiPublishersNode::on_timer_callback()
{
     convert_pcl_to_pointcloud2(point_cloud2_msg); // Create ROS2 Pointcloud2 Message
     convert_image_to_msg(*image_message_gray_left, img_pat_gray_left); // Create ROS2 Image Message
     ...
     prepare_navsatfix_msg(oxts_parsed_array , *nav_sat_fix_msg); // Create ROS2 NavSatFix Message
     prepare_imu_msg(oxts_parsed_array , *imu_msg); // Create ROS2 Imu Message
     prepare_marker_array_msg(oxts_parsed_array , *marker_array_msg); // Create ROS2 Marker Message, Convert latitude and longitude WGS84 to Cartesian.

     publish the messages...   
}
```

##  Visualize Rviz2 and Rqt

We use Rviz and Rqt for visualization. 
- You could import the "default.rviz" file into the Rviz2.
- You could import the "kitti_rqt.perspective" file into the Rqt.

## __Potential Improvements__

This project is an example study I developed for my ROS2 and C ++ learning process.
In CmakeList.txt, I will try the expressions one by one and discard the unnecessary ones.
I tried polymorphism by writing custom Publisher in C ++ classes, but since I could not use Node as an independent object, so I used lots of if-else blocks :)
You can send an e-mail to umtclskn@gmail.com for potential improvements, bugs, and incorrect implementations. Or you could open an issue here.
I've not calibrated the image messages yet. I'm planning to implement a ROS2 Service which will return thecamera calibration matrices.
Because I will calibrate camera images in a perception module I will implement.

# References
1. http://www.cvlibs.net/datasets/kitti/raw_data.php
2. https://github.com/iralabdisco/kitti_player
3. https://github.com/chrberger/WGS84toCartesian
4. https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp#L278
5. https://github.com/yanii/kitti-pcl
6. https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/

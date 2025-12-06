
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_KITTI_PUBLISHERS__KITTI_PUBLISHERS_NODE_HPP_
#define ROS2_KITTI_PUBLISHERS__KITTI_PUBLISHERS_NODE_HPP_

#include "rclcpp/rclcpp.hpp"


#include "std_msgs/msg/string.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <fstream>
#include <filesystem>
#include <vector>
#include <string>
#include <cstdlib>
#include <thread>
#include <future>
#include <atomic>

#include "ros2_kitti_publishers/visibility.h"
#include "ros2_kitti_publishers/WGS84toCartesian.hpp"
#include "ros2_kitti_publishers/file_manager.hpp"

class KittiPublishersNode : public rclcpp::Node
{
public:
  enum class PublisherType 
  { 
      POINT_CLOUD = 0,
      IMAGE_LEFT_GRAY = 1, 
      IMAGE_RIGHT_GRAY = 2,  
      IMAGE_LEFT_COLOR = 3, 
      IMAGE_RIGHT_COLOR = 4, 
      ODOMETRY = 5
  };

  KITTI_PUBLISHERS_NODE_PUBLIC KittiPublishersNode();

  std::string get_path(PublisherType publisher_type);
  std::vector<std::string> get_filenames(PublisherType publisher_type);
  void set_filenames(PublisherType publisher_type, std::vector<std::string> file_names);

private:
  void on_timer_callback();

  void init_file_path(const std::string& base_path);
  void create_publishers_data_file_names();
  void load_timestamps();
  rclcpp::Time parse_kitti_timestamp(const std::string& timestamp_str);
  std::vector<std::string> parse_file_data_into_string_array(std::string file_name, std::string delimiter);

  std::string mat_type2encoding(int mat_type);
  void convert_image_to_msg(sensor_msgs::msg::Image & msg, const std::string path );

  void prepare_navsatfix_msg(std::vector<std::string> &oxts_tokenized_array, sensor_msgs::msg::NavSatFix &msg);
  void prepare_imu_msg(std::vector<std::string> &oxts_tokenized_array, sensor_msgs::msg::Imu &msg);
  void prepare_marker_array_msg(std::vector<std::string> &oxts_tokenized_array, visualization_msgs::msg::MarkerArray &msg);
  void convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2 & msg, size_t file_index );
  
  size_t file_index_;
  std::atomic<bool> is_processing_;  // Flag to prevent overlapping timer callbacks
  
  // Statistics counters for debugging
  size_t published_image_color_right_count_;
  size_t skipped_image_color_right_count_;
  size_t published_image_gray_right_count_;
  size_t skipped_image_gray_right_count_;
  
  // FileManager for file operations (SRP refactoring)
  std::unique_ptr<ros2_kitti_publishers::FileManager> file_manager_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_;   // velodyne point clouds publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_gray_left_;     // left rectified grayscale image sequence
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_gray_right_;    // right rectified grayscale image sequence
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_color_left_;    // left rectified color image sequence
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_color_right_;   // right rectified color image sequence
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_odometry_;            // oxts odometry publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;                    // oxts odometry publisher
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_nav_sat_fix_;      // oxts odometry publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker_array_;  // oxts odometry publisher

  std::vector<std::string> file_names_point_cloud_;
  std::vector<std::string> file_names_image_gray_left_;
  std::vector<std::string> file_names_image_gray_right_;
  std::vector<std::string> file_names_image_color_left_;
  std::vector<std::string> file_names_image_color_right_;
  std::vector<std::string> file_names_oxts_;

  // Timestamp vectors for each data type
  std::vector<rclcpp::Time> timestamps_point_cloud_;
  std::vector<rclcpp::Time> timestamps_image_gray_left_;
  std::vector<rclcpp::Time> timestamps_image_gray_right_;
  std::vector<rclcpp::Time> timestamps_image_color_left_;
  std::vector<rclcpp::Time> timestamps_image_color_right_;
  std::vector<rclcpp::Time> timestamps_oxts_;

  rclcpp::Time dataset_start_time_;  // First timestamp (t0) from dataset

  std::string path_point_cloud_;
  std::string path_image_gray_left_;
  std::string path_image_gray_right_;
  std::string path_image_color_left_;
  std::string path_image_color_right_;
  std::string path_oxts_;
  std::string frame_id_;  // ROS2 frame ID for all messages
};

#endif  // ROS2_KITTI_PUBLISHERS__KITTI_PUBLISHERS_NODE_HPP_
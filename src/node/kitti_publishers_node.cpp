#include <chrono>
#include <sstream>
#include <iomanip>
#include <ctime>

#include "ros2_kitti_publishers/kitti_publishers_node.hpp"



using namespace cv;
using namespace std::chrono_literals;

KittiPublishersNode::KittiPublishersNode()
: Node("publisher_node"), file_index_(0), is_processing_(false)
{
  // Declare ROS2 parameters with default values
  this->declare_parameter<std::string>("dataset_base_path", "");
  this->declare_parameter<std::string>("frame_id", "base_link");
  this->declare_parameter<double>("publish_rate", 10.0);  // Hz

  // Get parameters
  std::string dataset_base_path = this->get_parameter("dataset_base_path").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  double publish_rate = this->get_parameter("publish_rate").as_double();

  RCLCPP_INFO(this->get_logger(), "Dataset base path: %s", dataset_base_path.c_str());
  RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publish rate: %.2f Hz", publish_rate);

  publisher_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti/point_cloud", 10);
  publisher_image_gray_left_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/left", 10);
  publisher_image_gray_right_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/right", 10);
  publisher_image_color_left_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/left", 10);
  publisher_image_color_right_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/right", 10);
  publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("kitti/imu", 10);
  publisher_nav_sat_fix_= this->create_publisher<sensor_msgs::msg::NavSatFix>("kitti/nav_sat_fix", 10);
  publisher_marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("kitti/marker_array", 10);

  init_file_path(dataset_base_path);

  create_publishers_data_file_names();
  
  // Load timestamps from KITTI dataset files
  load_timestamps();

  // Calculate timer period from rate
  auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));

  timer_ = create_wall_timer(
    timer_period, std::bind(&KittiPublishersNode::on_timer_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "KITTI Publishers Node initialized successfully");
}

void KittiPublishersNode::on_timer_callback()
{
    // Prevent overlapping callbacks - if previous callback is still processing, skip this one
    bool expected = false;
    if (!is_processing_.compare_exchange_strong(expected, true)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Previous callback still processing, skipping this timer tick");
        return;
    }

    // Check if we have any data files loaded
    if (file_names_point_cloud_.empty() && 
        file_names_image_gray_left_.empty() && 
        file_names_image_gray_right_.empty() &&
        file_names_image_color_left_.empty() &&
        file_names_image_color_right_.empty() &&
        file_names_oxts_.empty()) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            5000, 
            "No data files found. Please check your data paths configuration.");
        is_processing_ = false;
        return;
    }

    // Check bounds before accessing file vectors
    const size_t max_files = std::max({
        file_names_point_cloud_.size(),
        file_names_image_gray_left_.size(),
        file_names_image_gray_right_.size(),
        file_names_image_color_left_.size(),
        file_names_image_color_right_.size(),
        file_names_oxts_.size()
    });

    if (file_index_ >= max_files) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Reached end of dataset. Resetting to beginning.");
        file_index_ = 0;
        is_processing_ = false;
        return;
    }

    // Store current index to avoid race conditions
    const size_t current_index = file_index_;
    
    // Get real timestamps from KITTI dataset files
    // If timestamps are available, use them; otherwise use current time
    rclcpp::Time point_cloud_timestamp = this->now();
    rclcpp::Time image_gray_left_timestamp = this->now();
    rclcpp::Time image_gray_right_timestamp = this->now();
    rclcpp::Time image_color_left_timestamp = this->now();
    rclcpp::Time image_color_right_timestamp = this->now();
    rclcpp::Time oxts_timestamp = this->now();
    
    // Use real timestamps from dataset if available
    if (current_index < timestamps_point_cloud_.size()) {
        point_cloud_timestamp = timestamps_point_cloud_[current_index];
    }
    if (current_index < timestamps_image_gray_left_.size()) {
        image_gray_left_timestamp = timestamps_image_gray_left_[current_index];
    }
    if (current_index < timestamps_image_gray_right_.size()) {
        image_gray_right_timestamp = timestamps_image_gray_right_[current_index];
    }
    if (current_index < timestamps_image_color_left_.size()) {
        image_color_left_timestamp = timestamps_image_color_left_[current_index];
    }
    if (current_index < timestamps_image_color_right_.size()) {
        image_color_right_timestamp = timestamps_image_color_right_[current_index];
    }
    if (current_index < timestamps_oxts_.size()) {
        oxts_timestamp = timestamps_oxts_[current_index];
    }

    // Use async to read files in parallel
    auto point_cloud_future = std::async(std::launch::async, [this, current_index, point_cloud_timestamp]() {
        if (current_index < file_names_point_cloud_.size()) {
            sensor_msgs::msg::PointCloud2 msg;
            convert_pcl_to_pointcloud2(msg, current_index);
            msg.header.stamp = point_cloud_timestamp;  // Use real KITTI timestamp
            return std::make_optional(msg);
        }
        return std::optional<sensor_msgs::msg::PointCloud2>{};
    });

    auto image_gray_left_future = std::async(std::launch::async, [this, current_index, image_gray_left_timestamp]() {
        if (current_index < file_names_image_gray_left_.size()) {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            std::string path = path_image_gray_left_ + file_names_image_gray_left_[current_index];
            convert_image_to_msg(*msg, path);
            msg->header.stamp = image_gray_left_timestamp;  // Use real KITTI timestamp
            return msg;
        }
        return std::unique_ptr<sensor_msgs::msg::Image>{};
    });

    auto image_gray_right_future = std::async(std::launch::async, [this, current_index, image_gray_right_timestamp]() {
        if (current_index < file_names_image_gray_right_.size()) {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            std::string path = path_image_gray_right_ + file_names_image_gray_right_[current_index];
            convert_image_to_msg(*msg, path);
            msg->header.stamp = image_gray_right_timestamp;  // Use real KITTI timestamp
            return msg;
        }
        return std::unique_ptr<sensor_msgs::msg::Image>{};
    });

    auto image_color_left_future = std::async(std::launch::async, [this, current_index, image_color_left_timestamp]() {
        if (current_index < file_names_image_color_left_.size()) {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            std::string path = path_image_color_left_ + file_names_image_color_left_[current_index];
            convert_image_to_msg(*msg, path);
            msg->header.stamp = image_color_left_timestamp;  // Use real KITTI timestamp
            return msg;
        }
        return std::unique_ptr<sensor_msgs::msg::Image>{};
    });

    auto image_color_right_future = std::async(std::launch::async, [this, current_index, image_color_right_timestamp]() {
        if (current_index < file_names_image_color_right_.size()) {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            std::string path = path_image_color_right_ + file_names_image_color_right_[current_index];
            convert_image_to_msg(*msg, path);
            msg->header.stamp = image_color_right_timestamp;  // Use real KITTI timestamp
            return msg;
        }
        return std::unique_ptr<sensor_msgs::msg::Image>{};
    });

    // OXTS is small, can be read synchronously
    std::vector<std::string> oxts_parsed_array;
    if (current_index < file_names_oxts_.size()) {
        std::string oxts_file_name = path_oxts_ + file_names_oxts_[current_index];
        const std::string delimiter = " ";
        oxts_parsed_array = parse_file_data_into_string_array(oxts_file_name, delimiter);
    }

    // Wait for all async operations to complete and publish
    // 01- POINT CLOUD
    auto point_cloud_opt = point_cloud_future.get();
    if (point_cloud_opt.has_value()) {
        publisher_point_cloud_->publish(point_cloud_opt.value());
    }

    // 02- IMAGES
    auto img_gray_left = image_gray_left_future.get();
    if (img_gray_left) {
        publisher_image_gray_left_->publish(std::move(img_gray_left));
    }

    auto img_gray_right = image_gray_right_future.get();
    if (img_gray_right) {
        publisher_image_gray_right_->publish(std::move(img_gray_right));
    }

    auto img_color_left = image_color_left_future.get();
    if (img_color_left) {
        publisher_image_color_left_->publish(std::move(img_color_left));
    }

    auto img_color_right = image_color_right_future.get();
    if (img_color_right) {
        publisher_image_color_right_->publish(std::move(img_color_right));
    }

    // 03- OXTS MESSAGES (use real KITTI timestamp)
    if (!oxts_parsed_array.empty() && oxts_parsed_array.size() >= 30) {
        auto nav_sat_fix_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
        prepare_navsatfix_msg(oxts_parsed_array, *nav_sat_fix_msg);
        nav_sat_fix_msg->header.stamp = oxts_timestamp;  // Use real KITTI timestamp

        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
        prepare_imu_msg(oxts_parsed_array, *imu_msg);
        imu_msg->header.stamp = oxts_timestamp;  // Use real KITTI timestamp

        auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
        prepare_marker_array_msg(oxts_parsed_array, *marker_array_msg);
        // Set timestamp for all markers in the array
        for (auto& marker : marker_array_msg->markers) {
            marker.header.stamp = oxts_timestamp;
        }

        publisher_imu_->publish(std::move(imu_msg));
        publisher_nav_sat_fix_->publish(std::move(nav_sat_fix_msg));
        publisher_marker_array_->publish(std::move(marker_array_msg));
    }

    file_index_++;
    is_processing_ = false;
}

void KittiPublishersNode::convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2 & msg, size_t file_index ){
    // Initialize empty message first
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    
    // Bounds check
    if (file_index >= file_names_point_cloud_.size()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Point cloud file index %zu out of bounds (max: %zu)", 
            file_index, file_names_point_cloud_.size());
        return;
    }
    
    pcl::PointCloud<pcl::PointXYZI> cloud;
    std::string filePath = path_point_cloud_ + file_names_point_cloud_[file_index];
    
    std::ifstream input(filePath, std::ios::in | std::ios::binary);
    if(!input.is_open() || !input.good()){
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), 
          *this->get_clock(), 
          5000,
          "Could not read Velodyne's point cloud file: %s", filePath.c_str());
      return;
    }
    
    // Get file size to prevent reading beyond bounds
    input.seekg(0, std::ios::end);
    std::streampos file_size = input.tellg();
    input.seekg(0, std::ios::beg);
    
    // Each point is 4 floats (x, y, z, intensity) = 16 bytes
    const size_t point_size = 4 * sizeof(float);
    const size_t max_points = file_size / point_size;
    
    // Safety limit to prevent excessive memory usage
    const size_t max_safe_points = 200000; // ~3.2MB max
    size_t points_to_read = std::min(max_points, max_safe_points);
    
    cloud.reserve(points_to_read);
    
    for (size_t i = 0; i < points_to_read; i++) {
        pcl::PointXYZI point;
        
        // Read x, y, z (3 floats = 12 bytes)
        input.read(reinterpret_cast<char*>(&point.x), 3 * sizeof(float));
        
        // Check if we successfully read the data
        if (input.gcount() != 3 * sizeof(float)) {
            break; // End of file or read error
        }
        
        // Read intensity (1 float = 4 bytes)
        input.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
        
        if (input.gcount() != sizeof(float)) {
            break; // End of file or read error
        }
        
        cloud.push_back(point);
        
        // Check for EOF or errors
        if (input.eof() || input.fail()) {
            break;
        }
    }
    
    input.close();
    
    if (cloud.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Point cloud file is empty or could not be read: %s", filePath.c_str());
        return;
    }
    
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
}

void KittiPublishersNode::init_file_path(const std::string& base_path)
{
    // Ensure base_path ends with '/'
    std::string normalized_path = base_path;
    if (!normalized_path.empty() && normalized_path.back() != '/') {
        normalized_path += "/";
    }
    
    path_point_cloud_ = normalized_path + "velodyne_points/data/";
    path_image_gray_left_ = normalized_path + "image_00/data/";
    path_image_gray_right_ = normalized_path + "image_01/data/";
    path_image_color_left_ = normalized_path + "image_02/data/";
    path_image_color_right_ = normalized_path + "image_03/data/";
    path_oxts_ = normalized_path + "oxts/data/";
    
    RCLCPP_DEBUG(this->get_logger(), "Initialized paths from base: %s", normalized_path.c_str());
}

std::string KittiPublishersNode::get_path(KittiPublishersNode::PublisherType publisher_type)
{
  std::string path;
  if (publisher_type == KittiPublishersNode::PublisherType::POINT_CLOUD){
    path = path_point_cloud_;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_LEFT_GRAY){
    path = path_image_gray_left_;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_RIGHT_GRAY){
    path = path_image_gray_right_;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_LEFT_COLOR){
    path = path_image_color_left_;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_RIGHT_COLOR){
    path = path_image_color_right_;
  }else if(publisher_type == KittiPublishersNode::PublisherType::ODOMETRY){
    path = path_oxts_;
  }
  return path;
}

std::vector<std::string> KittiPublishersNode::get_filenames(PublisherType publisher_type)
{
  if (publisher_type == KittiPublishersNode::PublisherType::POINT_CLOUD){
     return file_names_point_cloud_;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_LEFT_GRAY){
     return file_names_image_gray_left_;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_RIGHT_GRAY){
     return file_names_image_gray_right_;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_LEFT_COLOR){
     return file_names_image_color_left_;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_RIGHT_COLOR){
     return file_names_image_color_right_;
  }
  return file_names_oxts_;
}

void KittiPublishersNode::set_filenames(PublisherType publisher_type, std::vector<std::string> file_names)
{
  if (publisher_type == KittiPublishersNode::PublisherType::POINT_CLOUD){
      file_names_point_cloud_= file_names;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_LEFT_GRAY){
      file_names_image_gray_left_= file_names;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_RIGHT_GRAY){
      file_names_image_gray_right_= file_names;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_LEFT_COLOR){
      file_names_image_color_left_= file_names;
  }else if(publisher_type == KittiPublishersNode::PublisherType::IMAGE_RIGHT_COLOR){
      file_names_image_color_right_ = file_names;
  }else if(publisher_type == KittiPublishersNode::PublisherType::ODOMETRY){
      file_names_oxts_= file_names;
  }
}

void KittiPublishersNode::create_publishers_data_file_names()
{
  bool at_least_one_path_found = false;
  
  for ( int type_index = 0; type_index != 6; type_index++ )
  {
    KittiPublishersNode::PublisherType type = static_cast<KittiPublishersNode::PublisherType>(type_index);
    std::vector<std::string> file_names = get_filenames(type);
    std::string path = get_path(type);

   try
   {
      if (std::filesystem::exists(path) && std::filesystem::is_directory(path)) {
        for (const auto & entry : std::filesystem::directory_iterator(path)){
          if (entry.is_regular_file()) {
              std::string filename = entry.path().filename().string();
              // Filter out Windows Zone.Identifier files and other hidden/system files
              if (filename.find("Zone.Identifier") == std::string::npos && 
                  filename[0] != '.') {
                  file_names.push_back(filename);
              }
          }
        }

        //Order file names
        std::sort(file_names.begin(), file_names.end(),
              [](const auto& lhs, const auto& rhs) {
                  return lhs  < rhs ;
              });
        set_filenames(type, file_names);
        
        if (!file_names.empty()) {
          at_least_one_path_found = true;
          RCLCPP_INFO(this->get_logger(), "Found %zu files in path: %s", file_names.size(), path.c_str());
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Path does not exist or is not a directory: %s", path.c_str());
      }
    }catch (const std::filesystem::filesystem_error& e)
    {
        RCLCPP_ERROR(this->get_logger(), "File path error for '%s': %s", path.c_str(), e.what());
    }
  }
  
  if (!at_least_one_path_found) {
    RCLCPP_ERROR(this->get_logger(), 
                 "No data files found in any configured path. Please check your data directory configuration.");
  }
}

rclcpp::Time KittiPublishersNode::parse_kitti_timestamp(const std::string& timestamp_str)
{
  // KITTI timestamp format: "2011-09-26 13:11:15.406628381"
  // Parse: YYYY-MM-DD HH:MM:SS.nanoseconds
  
  // Validate minimum length (at least 20 characters: "2011-09-26 13:11:15.")
  if (timestamp_str.length() < 20) {
    RCLCPP_WARN(this->get_logger(), "Timestamp string too short (expected at least 20 chars, got %zu): %s", 
                timestamp_str.length(), timestamp_str.c_str());
    return this->now();
  }
  
  std::tm tm = {};
  std::string date_time = timestamp_str.substr(0, 19);  // "2011-09-26 13:11:15"
  std::string nanoseconds_str;
  
  // Safely extract nanoseconds part (from position 20 to end)
  if (timestamp_str.length() > 20) {
    nanoseconds_str = timestamp_str.substr(20); // "406628381"
  } else {
    // If exactly 20 characters, no nanoseconds part
    nanoseconds_str = "0";
    RCLCPP_DEBUG(this->get_logger(), "No nanoseconds in timestamp, using 0: %s", timestamp_str.c_str());
  }
  
  // Parse date and time
  std::istringstream ss(date_time);
  ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
  
  if (ss.fail()) {
    RCLCPP_WARN(this->get_logger(), "Failed to parse timestamp: %s", timestamp_str.c_str());
    return this->now();
  }
  
  // Convert to Unix timestamp (seconds since 1970-01-01)
  // KITTI timestamps are in UTC
  // Use reference epoch: 2011-09-26 00:00:00 UTC = 1316995200 seconds since Unix epoch
  const int64_t reference_epoch = 1316995200LL;
  
  // Calculate time difference from reference date (2011-09-26)
  int year = tm.tm_year + 1900;
  int month = tm.tm_mon + 1;  // tm_mon is 0-11, so add 1
  int day = tm.tm_mday;
  
  // Calculate days from 2011-09-26
  int64_t days_diff = 0;
  if (year == 2011 && month >= 9) {
    // Days from September 26
    int days_in_sep = day - 26;
    // Add days for months after September (Oct, Nov, Dec)
    int days_in_months[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    for (int m = 10; m <= month; m++) {
      days_diff += days_in_months[m];
    }
    days_diff += days_in_sep;
  }
  
  // Calculate total seconds from reference
  int64_t total_seconds = reference_epoch + (days_diff * 86400LL) + 
                          (tm.tm_hour * 3600LL) + (tm.tm_min * 60LL) + tm.tm_sec;
  
  // Add nanoseconds
  int64_t nanoseconds = 0;
  try {
    nanoseconds = std::stoll(nanoseconds_str);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Failed to parse nanoseconds: %s", nanoseconds_str.c_str());
  }
  
  // Convert to ROS2 Time (nanoseconds since Unix epoch)
  int64_t total_nanoseconds = total_seconds * 1000000000LL + nanoseconds;
  
  return rclcpp::Time(total_nanoseconds, RCL_ROS_TIME);
}

void KittiPublishersNode::load_timestamps()
{
  RCLCPP_INFO(this->get_logger(), "Loading timestamps from KITTI dataset...");
  
  // Helper lambda to load timestamps from a file
  auto load_timestamp_file = [this](const std::string& file_path, std::vector<rclcpp::Time>& timestamps) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Could not open timestamp file: %s", file_path.c_str());
      return false;
    }
    
    std::string line;
    size_t line_number = 0;
    size_t valid_timestamps = 0;
    size_t invalid_timestamps = 0;
    
    while (std::getline(file, line)) {
      line_number++;
      
      // Remove trailing whitespace and newlines
      line.erase(0, line.find_first_not_of(" \t\r\n"));
      line.erase(line.find_last_not_of(" \t\r\n") + 1);
      
      if (line.empty() || line.find("Zone.Identifier") != std::string::npos) {
        continue; // Skip empty lines and Zone.Identifier files
      }
      
      // Parse timestamp with exception handling
      try {
        rclcpp::Time ts = parse_kitti_timestamp(line);
        timestamps.push_back(ts);
        valid_timestamps++;
      } catch (const std::out_of_range& e) {
        invalid_timestamps++;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "Out of range error parsing timestamp at line %zu in %s: %s (error: %s)", 
          line_number, file_path.c_str(), line.c_str(), e.what());
      } catch (const std::exception& e) {
        invalid_timestamps++;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "Failed to parse timestamp at line %zu in %s: %s (error: %s)", 
          line_number, file_path.c_str(), line.c_str(), e.what());
      }
    }
    file.close();
    
    if (invalid_timestamps > 0) {
      RCLCPP_WARN(this->get_logger(), 
                  "Loaded %zu valid timestamps, skipped %zu invalid timestamps from %s",
                  valid_timestamps, invalid_timestamps, file_path.c_str());
    }
    
    return valid_timestamps > 0; // Return true if at least one valid timestamp was loaded
  };
  
  // Load timestamps for each data type
  bool loaded_any = false;
  
  if (!file_names_point_cloud_.empty()) {
    std::string timestamp_file = path_point_cloud_ + "../timestamps.txt";
    if (load_timestamp_file(timestamp_file, timestamps_point_cloud_)) {
      RCLCPP_INFO(this->get_logger(), "Loaded %zu point cloud timestamps", timestamps_point_cloud_.size());
      loaded_any = true;
    }
  }
  
  if (!file_names_image_gray_left_.empty()) {
    std::string timestamp_file = path_image_gray_left_ + "../timestamps.txt";
    if (load_timestamp_file(timestamp_file, timestamps_image_gray_left_)) {
      RCLCPP_INFO(this->get_logger(), "Loaded %zu image gray left timestamps", timestamps_image_gray_left_.size());
      loaded_any = true;
    }
  }
  
  if (!file_names_image_gray_right_.empty()) {
    std::string timestamp_file = path_image_gray_right_ + "../timestamps.txt";
    if (load_timestamp_file(timestamp_file, timestamps_image_gray_right_)) {
      RCLCPP_INFO(this->get_logger(), "Loaded %zu image gray right timestamps", timestamps_image_gray_right_.size());
      loaded_any = true;
    }
  }
  
  if (!file_names_image_color_left_.empty()) {
    std::string timestamp_file = path_image_color_left_ + "../timestamps.txt";
    if (load_timestamp_file(timestamp_file, timestamps_image_color_left_)) {
      RCLCPP_INFO(this->get_logger(), "Loaded %zu image color left timestamps", timestamps_image_color_left_.size());
      loaded_any = true;
    }
  }
  
  if (!file_names_image_color_right_.empty()) {
    std::string timestamp_file = path_image_color_right_ + "../timestamps.txt";
    if (load_timestamp_file(timestamp_file, timestamps_image_color_right_)) {
      RCLCPP_INFO(this->get_logger(), "Loaded %zu image color right timestamps", timestamps_image_color_right_.size());
      loaded_any = true;
    }
  }
  
  if (!file_names_oxts_.empty()) {
    std::string timestamp_file = path_oxts_ + "../timestamps.txt";
    if (load_timestamp_file(timestamp_file, timestamps_oxts_)) {
      RCLCPP_INFO(this->get_logger(), "Loaded %zu OXTS timestamps", timestamps_oxts_.size());
      loaded_any = true;
    }
  }
  
  // Find the earliest timestamp (t0) from all loaded timestamps
  if (loaded_any) {
    bool first_found = false;
    rclcpp::Time earliest(0, 0, RCL_ROS_TIME);  // Initialize with same time source
    
    // Find the first available timestamp to use as initial value
    if (!timestamps_point_cloud_.empty()) {
      earliest = timestamps_point_cloud_[0];
      first_found = true;
    } else if (!timestamps_image_gray_left_.empty()) {
      earliest = timestamps_image_gray_left_[0];
      first_found = true;
    } else if (!timestamps_image_gray_right_.empty()) {
      earliest = timestamps_image_gray_right_[0];
      first_found = true;
    } else if (!timestamps_image_color_left_.empty()) {
      earliest = timestamps_image_color_left_[0];
      first_found = true;
    } else if (!timestamps_image_color_right_.empty()) {
      earliest = timestamps_image_color_right_[0];
      first_found = true;
    } else if (!timestamps_oxts_.empty()) {
      earliest = timestamps_oxts_[0];
      first_found = true;
    }
    
    // Now compare with all timestamps to find the earliest
    if (first_found) {
      if (!timestamps_point_cloud_.empty() && timestamps_point_cloud_[0] < earliest) {
        earliest = timestamps_point_cloud_[0];
      }
      if (!timestamps_image_gray_left_.empty() && timestamps_image_gray_left_[0] < earliest) {
        earliest = timestamps_image_gray_left_[0];
      }
      if (!timestamps_image_gray_right_.empty() && timestamps_image_gray_right_[0] < earliest) {
        earliest = timestamps_image_gray_right_[0];
      }
      if (!timestamps_image_color_left_.empty() && timestamps_image_color_left_[0] < earliest) {
        earliest = timestamps_image_color_left_[0];
      }
      if (!timestamps_image_color_right_.empty() && timestamps_image_color_right_[0] < earliest) {
        earliest = timestamps_image_color_right_[0];
      }
      if (!timestamps_oxts_.empty() && timestamps_oxts_[0] < earliest) {
        earliest = timestamps_oxts_[0];
      }
    }
    
    dataset_start_time_ = earliest;
    RCLCPP_INFO(this->get_logger(), "Dataset start time (t0): %.9f", 
                dataset_start_time_.seconds());
  } else {
    RCLCPP_WARN(this->get_logger(), "No timestamps loaded. Using current time as reference.");
    dataset_start_time_ = this->now();
  }
}


void KittiPublishersNode::prepare_navsatfix_msg(std::vector<std::string> &oxts_tokenized_array, sensor_msgs::msg::NavSatFix &msg)
{
  msg.header.frame_id = frame_id_;
  // Note: timestamp will be set by caller to ensure synchronization

  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  msg.status.status  = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;

  msg.latitude  = std::atof(oxts_tokenized_array[0].c_str());
  msg.longitude = std::atof(oxts_tokenized_array[1].c_str());
  msg.altitude  = std::atof(oxts_tokenized_array[2].c_str());

  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
  msg.position_covariance[0] = std::atof(oxts_tokenized_array[23].c_str());
  msg.position_covariance[1] = 0.0f;
  msg.position_covariance[2] = 0.0f;
  msg.position_covariance[3] = 0.0f;
  msg.position_covariance[4] = std::atof(oxts_tokenized_array[23].c_str());
  msg.position_covariance[5] = 0.0f;
  msg.position_covariance[6] = 0.0f;
  msg.position_covariance[7] = 0.0f;
  msg.position_covariance[8] = std::atof(oxts_tokenized_array[23].c_str());
}

// https://github.com/iralabdisco/kitti_player/blob/public/src/kitti_player.cpp#L1252
// https://github.com/chrberger/WGS84toCartesian
void KittiPublishersNode::prepare_marker_array_msg(std::vector<std::string> &oxts_tokenized_array, visualization_msgs::msg::MarkerArray &msg)
{
  const double lat =  std::stod(oxts_tokenized_array[0]);
  const double lon =  std::stod(oxts_tokenized_array[1]);
  
  std::array<double, 2> WGS84Reference{lat, lon};
  std::array<double, 2> WGS84Position{lat, lon};
  std::array<double, 2> result{wgs84::toCartesian(WGS84Reference, WGS84Position)};

  visualization_msgs::msg::Marker RTK_MARKER;

  static int gps_track = 1;
  RTK_MARKER.header.frame_id = frame_id_;
  // Note: timestamp will be set by caller to ensure synchronization
  RTK_MARKER.ns = "RTK_MARKER";
  RTK_MARKER.id = gps_track++; //unused
  RTK_MARKER.type = visualization_msgs::msg::Marker::CYLINDER;
  RTK_MARKER.action = visualization_msgs::msg::Marker::ADD;
  RTK_MARKER.pose.orientation.w = 1;
  RTK_MARKER.scale.x = 0.5;
  RTK_MARKER.scale.y = 0.5;
  RTK_MARKER.scale.z = 3.5;
  RTK_MARKER.color.a = 0.80;
  RTK_MARKER.color.r = 0;
  RTK_MARKER.color.g = 0.0;
  RTK_MARKER.color.b = 1.0;
  RTK_MARKER.pose.position.x = result[0];
  RTK_MARKER.pose.position.y = result[1];
  RTK_MARKER.pose.position.z = 0;

  msg.markers.push_back(RTK_MARKER);
}

// https://github.com/iralabdisco/kitti_player/blob/public/src/kitti_player.cpp
void KittiPublishersNode::prepare_imu_msg(std::vector<std::string> &oxts_tokenized_array, sensor_msgs::msg::Imu &msg){
  msg.header.frame_id = frame_id_;
  // Note: timestamp will be set by caller to ensure synchronization

  //    - ax:      acceleration in x, i.e. in direction of vehicle front (m/s^2)
  //    - ay:      acceleration in y, i.e. in direction of vehicle left (m/s^2)
  //    - az:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
  msg.linear_acceleration.x = std::atof(oxts_tokenized_array[11].c_str());
  msg.linear_acceleration.y = std::atof(oxts_tokenized_array[12].c_str());
  msg.linear_acceleration.z = std::atof(oxts_tokenized_array[13].c_str());

  //    - vf:      forward velocity, i.e. parallel to earth-surface (m/s)
  //    - vl:      leftward velocity, i.e. parallel to earth-surface (m/s)
  //    - vu:      upward velocity, i.e. perpendicular to earth-surface (m/s)
  msg.angular_velocity.x = std::atof(oxts_tokenized_array[8].c_str());
  msg.angular_velocity.y = std::atof(oxts_tokenized_array[9].c_str());
  msg.angular_velocity.z = std::atof(oxts_tokenized_array[10].c_str());

  //    - roll:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
  //    - pitch:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
  //    - yaw:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
  tf2::Quaternion q;
  q.setRPY(std::atof(oxts_tokenized_array[3].c_str()), 
            std::atof(oxts_tokenized_array[4].c_str()), 
            std::atof(oxts_tokenized_array[5].c_str()));

  msg.orientation.x = q.getX();
  msg.orientation.y = q.getY();
  msg.orientation.z = q.getZ();
  msg.orientation.w = q.getW();
}

//https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp#L278
void KittiPublishersNode::convert_image_to_msg(sensor_msgs::msg::Image & msg, const std::string path  )
{
  Mat frame;
  frame = imread(path);
  if (frame.empty())                      // Check for invalid input
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), 
        *this->get_clock(), 
        5000,
        "Image does not exist or could not be read: %s", path.c_str());
    // Return empty message
    msg.header.frame_id = frame_id_;
    // Note: timestamp will be set by caller to ensure synchronization
    return;
  }

  msg.height = frame.rows;
  msg.width = frame.cols;
  std::string type = mat_type2encoding(frame.type());
  msg.encoding = type;
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = frame_id_;
  // Note: timestamp will be set by caller to ensure synchronization
}

std::string KittiPublishersNode::mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

std::vector<std::string> KittiPublishersNode::parse_file_data_into_string_array(std::string file_name, std::string delimiter)
{
    std::vector<std::string> tokens;
    std::ifstream f(file_name.c_str()); //taking file as inputstream

    if(!f.is_open() || !f.good()){
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), 
          *this->get_clock(), 
          5000,
          "Could not read OXTS data file: %s", file_name.c_str());
      return tokens; // Return empty vector
    }

    std::string file_content_string;
    if(f) {
        std::ostringstream ss;
        ss << f.rdbuf(); // reading data
        file_content_string = ss.str();
    }
    f.close();

    // Remove trailing newlines and whitespace
    while (!file_content_string.empty() && 
           (file_content_string.back() == '\n' || 
            file_content_string.back() == '\r' || 
            file_content_string.back() == ' ')) {
        file_content_string.pop_back();
    }

    //https://www.codegrepper.com/code-examples/whatever/c%2B%2B+how+to+tokenize+a+string  
    size_t first = 0;
    while(first < file_content_string.size()){
        size_t second = file_content_string.find_first_of(delimiter, first);
        //first has index of start of token
        //second has index of end of token + 1;
        if(second == std::string::npos){
            second = file_content_string.size();
        }
        
        // Only add non-empty tokens
        if (second > first) {
            std::string token = file_content_string.substr(first, second-first);
            // Remove any remaining whitespace from token
            token.erase(0, token.find_first_not_of(" \t\r\n"));
            token.erase(token.find_last_not_of(" \t\r\n") + 1);
            if (!token.empty()) {
                tokens.push_back(token);
            }
        }
        first = second + 1;
    }

    return tokens;
}

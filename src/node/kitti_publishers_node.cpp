#include <chrono>

#include "ros2_kitti_publishers/kitti_publishers_node.hpp"



using namespace cv;
using namespace std::chrono_literals;

KittiPublishersNode::KittiPublishersNode()
: Node("publisher_node"), file_index_(0)
{

  publisher_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti/point_cloud", 10);
  publisher_image_gray_left_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/left", 10);
  publisher_image_gray_right_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/right", 10);
  publisher_image_color_left_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/left", 10);
  publisher_image_color_right_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/right", 10);
  publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("kitti/imu", 10);
  publisher_nav_sat_fix_= this->create_publisher<sensor_msgs::msg::NavSatFix>("kitti/nav_sat_fix", 10);
  publisher_marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("kitti/marker_array", 10);

  init_file_path();

  create_publishers_data_file_names();

  timer_ = create_wall_timer(
    100ms, std::bind(&KittiPublishersNode::on_timer_callback, this));
}

void KittiPublishersNode::on_timer_callback()
{
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
        return;
    }

    // 01- KITTI POINT CLOUDS2 MESSAGES START//
    if (file_index_ < file_names_point_cloud_.size()) {
        sensor_msgs::msg::PointCloud2 point_cloud2_msg;
        convert_pcl_to_pointcloud2(point_cloud2_msg);
        publisher_point_cloud_->publish(point_cloud2_msg);
    }
    // 01- KITTI POINT CLOUDS2 MESSAGES END//

    // 02- KITTI IMAGE MESSAGES START- gray_left(image_00), gray_right(image_01), color_left(image_02), color_right(image_03)//   
    if (file_index_ < file_names_image_gray_left_.size()) {
        auto image_message_gray_left = std::make_unique<sensor_msgs::msg::Image>();
        std::string img_pat_gray_left = path_image_gray_left_ + file_names_image_gray_left_[file_index_];
        convert_image_to_msg(*image_message_gray_left, img_pat_gray_left);
        publisher_image_gray_left_->publish(std::move(image_message_gray_left));
    }

    if (file_index_ < file_names_image_gray_right_.size()) {
        auto image_message_gray_right = std::make_unique<sensor_msgs::msg::Image>();
        std::string img_pat_gray_right = path_image_gray_right_ + file_names_image_gray_right_[file_index_];
        convert_image_to_msg(*image_message_gray_right, img_pat_gray_right);
        publisher_image_gray_right_->publish(std::move(image_message_gray_right));
    }

    if (file_index_ < file_names_image_color_left_.size()) {
        auto image_message_color_left = std::make_unique<sensor_msgs::msg::Image>();
        std::string img_pat_color_left = path_image_color_left_ + file_names_image_color_left_[file_index_];
        convert_image_to_msg(*image_message_color_left, img_pat_color_left);
        publisher_image_color_left_->publish(std::move(image_message_color_left));
    }

    if (file_index_ < file_names_image_color_right_.size()) {
        auto image_message_color_right = std::make_unique<sensor_msgs::msg::Image>();
        std::string img_pat_color_right = path_image_color_right_ + file_names_image_color_right_[file_index_];
        convert_image_to_msg(*image_message_color_right, img_pat_color_right);
        publisher_image_color_right_->publish(std::move(image_message_color_right));
    }
    // 02- KITTI IMAGE MESSAGES END // 

    // 03- KITTI OXTS to IMU, NAV & MARKERARRAY MESSAGE START//
    if (file_index_ < file_names_oxts_.size()) {
        std::string oxts_file_name = path_oxts_ + file_names_oxts_[file_index_];
        const std::string delimiter = " ";
        std::vector<std::string> oxts_parsed_array = parse_file_data_into_string_array(oxts_file_name, delimiter);
        
        // Check if we have enough data (OXTS files should have at least 30 fields)
        if (oxts_parsed_array.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "OXTS file is empty or could not be parsed: %s", oxts_file_name.c_str());
        } else if (oxts_parsed_array.size() < 30) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "OXTS data incomplete. Expected at least 30 fields, got %zu. File: %s", 
                oxts_parsed_array.size(), oxts_file_name.c_str());
        } else {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "OxTs size: '%zu' from file: %s", 
                oxts_parsed_array.size(), oxts_file_name.c_str());

            auto nav_sat_fix_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
            prepare_navsatfix_msg(oxts_parsed_array , *nav_sat_fix_msg);

            auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
            prepare_imu_msg(oxts_parsed_array , *imu_msg);

            auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
            prepare_marker_array_msg(oxts_parsed_array , *marker_array_msg);

            publisher_imu_->publish(std::move(imu_msg));
            publisher_nav_sat_fix_->publish(std::move(nav_sat_fix_msg));
            publisher_marker_array_->publish(std::move(marker_array_msg));
        }
    }
    // 03- KITTI OXTS to IMU, NAV & MARKERARRAY MESSAGE END//

    file_index_++;
}

void KittiPublishersNode::convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2 & msg ){
    // Initialize empty message first
    msg.header.frame_id = "base_link";
    msg.header.stamp = now();
    
    // Bounds check
    if (file_index_ >= file_names_point_cloud_.size()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Point cloud file index %zu out of bounds (max: %zu)", 
            file_index_, file_names_point_cloud_.size());
        return;
    }
    
    pcl::PointCloud<pcl::PointXYZI> cloud;
    std::string filePath = path_point_cloud_ + file_names_point_cloud_[file_index_];
    
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
    msg.header.frame_id = "base_link";
    msg.header.stamp = now();
}

void KittiPublishersNode::init_file_path()
{
    // Updated path for drive_0014_sync dataset (WSL filesystem path)
    const std::string base_path = "/home/umut/kitti/2011_09_26/2011_09_26_drive_0014_sync/";
    path_point_cloud_ = base_path + "velodyne_points/data/";
    path_image_gray_left_ = base_path + "image_00/data/";
    path_image_gray_right_ = base_path + "image_01/data/";
    path_image_color_left_ = base_path + "image_02/data/";
    path_image_color_right_ = base_path + "image_03/data/";
    path_oxts_ = base_path + "oxts/data/";
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


void KittiPublishersNode::prepare_navsatfix_msg(std::vector<std::string> &oxts_tokenized_array, sensor_msgs::msg::NavSatFix &msg)
{
  msg.header.frame_id = "base_link";
  msg.header.stamp = this->now();

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
  RTK_MARKER.header.frame_id = "base_link";
  RTK_MARKER.header.stamp = this->now();
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
  msg.header.frame_id = "base_link";
  msg.header.stamp = now();

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
    msg.header.frame_id = "base_link";
    msg.header.stamp = this->now();
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
  msg.header.frame_id = "base_link";
  msg.header.stamp = this->now();
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

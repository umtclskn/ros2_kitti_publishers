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
    // 01- KITTI POINT CLOUDS2 MESSAGES START//
    sensor_msgs::msg::PointCloud2 point_cloud2_msg;
    convert_pcl_to_pointcloud2(point_cloud2_msg);
    // 01- KITTI POINT CLOUDS2 MESSAGES END//

    // 02- KITTI IMAGE MESSAGES START- gray_left(image_00), gray_right(image_01), color_left(image_02), color_right(image_03)//   
    auto image_message_gray_left = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_gray_left = path_image_gray_left_ + file_names_image_color_left_[file_index_];
    convert_image_to_msg(*image_message_gray_left, img_pat_gray_left);

    auto image_message_gray_right = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_gray_right = path_image_gray_right_ + file_names_image_color_right_[file_index_];
    convert_image_to_msg(*image_message_gray_right, img_pat_gray_right);

    auto image_message_color_left = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_color_left = path_image_color_right_ + file_names_image_color_left_[file_index_];
    convert_image_to_msg(*image_message_color_left, img_pat_color_left);

    auto image_message_color_right = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_color_right = path_image_color_right_ + file_names_image_color_right_[file_index_];
    convert_image_to_msg(*image_message_color_right, img_pat_color_right);
    // 02- KITTI IMAGE MESSAGES END // 

    // 03- KITTI OXTS to IMU, NAV & MARKERARRAY MESSAGE START//
    std::string oxts_file_name = path_oxts_ + file_names_oxts_[file_index_];
    const std::string delimiter = " ";
    std::vector<std::string> oxts_parsed_array = parse_file_data_into_string_array(oxts_file_name, delimiter);
    RCLCPP_INFO(this->get_logger(), "OxTs size: '%i'", oxts_parsed_array.size());

    auto nav_sat_fix_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
    prepare_navsatfix_msg(oxts_parsed_array , *nav_sat_fix_msg);

    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    prepare_imu_msg(oxts_parsed_array , *imu_msg);

    auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
    prepare_marker_array_msg(oxts_parsed_array , *marker_array_msg);
    // 03- KITTI OXTS to IMU, NAV & MARKERARRAY MESSAGE END//

    publisher_point_cloud_->publish(point_cloud2_msg);
    publisher_image_gray_left_->publish(std::move(image_message_gray_left));
    publisher_image_gray_right_->publish(std::move(image_message_gray_right));
    publisher_image_color_left_->publish(std::move(image_message_color_left));
    publisher_image_color_right_->publish(std::move(image_message_color_right));

    publisher_imu_->publish(std::move(imu_msg));
    publisher_nav_sat_fix_->publish(std::move(nav_sat_fix_msg));
    publisher_marker_array_->publish(std::move(marker_array_msg));

    file_index_++;
}

void KittiPublishersNode::convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2 & msg ){
    pcl::PointCloud<pcl::PointXYZI> cloud;

    std::string filePath = get_path(KittiPublishersNode::PublisherType::POINT_CLOUD) + file_names_point_cloud_[file_index_];
    std::fstream input(filePath, std::ios::in | std::ios::binary);
    if(!input.good()){
      RCLCPP_INFO(this->get_logger(), "Could not read Velodyne's point cloud. Check your file path!");
      exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    for (int i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud.push_back(point);
    }

    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "base_link";
    msg.header.stamp = now();
}

void KittiPublishersNode::init_file_path()
{
    path_point_cloud_ = "data/2011_09_26/2011_09_26_drive_0015_sync/velodyne_points/data/";
    path_image_gray_left_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_00/data/";
    path_image_gray_right_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_01/data/";
    path_image_color_left_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_02/data/";
    path_image_color_right_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_03/data/";
    path_oxts_ = "data/2011_09_26/2011_09_26_drive_0015_sync/oxts/data/";
}

std::string KittiPublishersNode::get_path(KittiPublishersNode::PublisherType publisher_type)
{
  RCLCPP_INFO(this->get_logger(), "get_path: '%i'", publisher_type);
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
  for ( int type_index = 0; type_index != 6; type_index++ )
  {
    KittiPublishersNode::PublisherType type = static_cast<KittiPublishersNode::PublisherType>(type_index);
    std::vector<std::string> file_names = get_filenames(type);

   try
   {
      for (const auto & entry : std::filesystem::directory_iterator(get_path(type))){
        if (entry.is_regular_file()) {
            file_names.push_back(entry.path().filename());
        }
      }

      //Order lidar file names
      std::sort(file_names.begin(), file_names.end(),
            [](const auto& lhs, const auto& rhs) {
                return lhs  < rhs ;
            });
      set_filenames(type, file_names);
    }catch (const std::filesystem::filesystem_error& e)
    {
        RCLCPP_ERROR(this->get_logger(), "File path not found.");
    }
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
    RCLCPP_ERROR(this->get_logger(), "Image does not exist. Check your files path!");
    rclcpp::shutdown();
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
    std::ifstream f(file_name.c_str()); //taking file as inputstream

    if(!f.good()){
      RCLCPP_INFO(this->get_logger(), "Could not read OXTS data. Check your file path!");
      exit(EXIT_FAILURE);
    }

    std::string file_content_string;
    if(f) {
        std::ostringstream ss;
        ss << f.rdbuf(); // reading data
        file_content_string = ss.str();
    }

    //https://www.codegrepper.com/code-examples/whatever/c%2B%2B+how+to+tokenize+a+string  
    std::vector<std::string> tokens;
    size_t first = 0;
    while(first < file_content_string.size()){
        size_t second = file_content_string.find_first_of(delimiter,first);
        //first has index of start of token
        //second has index of end of token + 1;
        if(second == std::string::npos){
            second = file_content_string.size();
        }
        std::string token = file_content_string.substr(first, second-first);
        tokens.push_back(token);
        first = second + 1;
    }

    return tokens;
}

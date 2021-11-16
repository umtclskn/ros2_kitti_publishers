#include <chrono>
#include <iostream>
#include <tf2/LinearMath/Transform.h>
#include "ros2_kitti_publishers/kitti_publishers_node.hpp"

using namespace std;

using namespace cv;
using namespace std::chrono_literals;

// WGS 84
const double semimajor_axis = 6378137.0;
const double semiminor_axis = 6356752.31424518;

KittiPublishersNode::KittiPublishersNode()
: Node("publisher_node"), file_index_(0)
{
  imu2velo_tf_bcaster  = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  odom2base_tf_bcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  publisher_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti/point_cloud", 10);
  publisher_image_gray_left_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/left", 10);
  publisher_image_gray_right_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/right", 10);
  publisher_image_color_left_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/left", 10);
  publisher_image_color_right_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/right", 10);
  publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("kitti/imu", 10);
  publisher_nav_sat_fix_= this->create_publisher<sensor_msgs::msg::NavSatFix>("kitti/nav_sat_fix", 10);
  publisher_marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("kitti/marker_array", 10);
  publisher_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("kitti/odom", 10);

  // imu to lidar calibration parameters
  // I don't need camera data, so no camera relevant calibration parameter loaded
  rclcpp::Parameter param;
  std::vector<double> vec;
  this->declare_parameter("imu2velo_translation", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->get_parameter("imu2velo_translation", param);
  vec = param.as_double_array();
  imu2velo_tf_msg.transform.translation.x = vec[0];
  imu2velo_tf_msg.transform.translation.y = vec[1];
  imu2velo_tf_msg.transform.translation.z = vec[2];

  this->declare_parameter("imu2velo_rotation", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  this->get_parameter("imu2velo_rotation", param);
  vec = param.as_double_array();
  tf2::Matrix3x3 mmm(vec[0], vec[1], vec[2],
                        vec[3], vec[4], vec[5],
                        vec[6], vec[7], vec[8]);

  tf2::Transform tfm(mmm);
  tf2::Quaternion q = tfm.getRotation();
  imu2velo_tf_msg.transform.rotation.x = q.x();
  imu2velo_tf_msg.transform.rotation.y = q.y();
  imu2velo_tf_msg.transform.rotation.z = q.z();
  imu2velo_tf_msg.transform.rotation.w = q.w();
  imu2velo_tf_msg.header.frame_id = "base_link";
  imu2velo_tf_msg.child_frame_id = "velodyne";

  // path parameters
  const std::vector<std::string> param_names = {"data_file_root_path", "data_file_date", "data_file_option", "data_file_sensors"};
  for(unsigned int i=0; i < param_names.size()-1; ++i) {
    this->declare_parameter(param_names[i], rclcpp::ParameterType::PARAMETER_STRING);
  }
  this->declare_parameter(param_names.back(), rclcpp::ParameterType::PARAMETER_STRING_ARRAY);

  std::vector<std::string> prefices;
  std::vector<std::string> sensors;
  for(unsigned int i=0; i < param_names.size()-1; ++i){
    this->get_parameter(param_names[i], param);
    prefices.push_back(param.value_to_string());
  }
  std::string prefix = prefices[0] + "/" + prefices[1] + "/" + prefices[1] + prefices[2] + "/";
  this->get_parameter(param_names.back(), param);
  prefices.clear();
  prefices = param.as_string_array();

  path_point_cloud_ = prefix + prefices[0];
  path_image_gray_left_ = prefix + prefices[1];
  path_image_gray_right_ = prefix + prefices[2];
  path_image_color_left_ = prefix + prefices[3];
  path_image_color_right_ = prefix + prefices[4];
  path_oxts_ = prefix + prefices[5];

  // init_file_path();
  odom_inited_ = false;

  create_publishers_data_file_names();

  timer_ = create_wall_timer(
    100ms, std::bind(&KittiPublishersNode::on_timer_callback, this));

  // timestamp file
  ifs_lidar_stamp.open(path_point_cloud_ + "/timestamps.txt");
  // std::cout << path_point_cloud_ << "/timestamp.txt" << std::endl;
  ifs_image_gray_left_stamp.open(path_image_gray_left_ + "/timestamps.txt");
  ifs_image_gray_right_stamp.open(path_image_gray_right_ + "/timestamps.txt");
  ifs_image_color_left_stamp.open(path_image_color_left_  + "/timestamps.txt");
  ifs_image_color_right_stamp.open(path_image_color_right_ + "/timestamps.txt");
  ifs_oxts_stamp.open(path_oxts_ + "/timestamps.txt");

  // KITTI data is recorded in Central European Timezone
  // you should set the environment variable
  tzset();
}

void KittiPublishersNode::utc2ts(std::string &utc, builtin_interfaces::msg::Time &timestamp) {
  struct tm t;
  time_t sec;
  char delimeter = '.';
  std::string utci;
  std::stringstream utcs{utc};

  // secs
  std::getline(utcs, utci, delimeter);
  strptime(utci.c_str(), "%Y-%m-%d %H:%M:%S", &t);
  t.tm_isdst = 0;
  sec = mktime(&t);
  assert(sec != -1);
  timestamp.sec = sec;

  // and nsecs
  std::getline(utcs, utci, delimeter);
  int nsec = stoi(utci);
  timestamp.nanosec = nsec;
}

void KittiPublishersNode::on_timer_callback()
{
  if(file_names_point_cloud_.size() < 5)
    return;
  std::string line;
  builtin_interfaces::msg::Time timestamp;
  const KittiExceptions e = END_OF_FILE;

  // 01- KITTI POINT CLOUDS2 MESSAGES START//
  sensor_msgs::msg::PointCloud2 point_cloud2_msg;
  std::string path_lidar = get_path(KittiPublishersNode::PublisherType::POINT_CLOUD) + "/data/" + file_names_point_cloud_[file_index_];
  getline(ifs_lidar_stamp, line);
  if(ifs_lidar_stamp.eof())
    throw(e);
  utc2ts(line, timestamp);
  convert_pcl_to_pointcloud2(point_cloud2_msg, path_lidar, timestamp);
  // 01- KITTI POINT CLOUDS2 MESSAGES END//

  // 02- KITTI IMAGE MESSAGES START- gray_left(image_00), gray_right(image_01), color_left(image_02), color_right(image_03)//
  auto image_message_gray_left = std::make_unique<sensor_msgs::msg::Image>();
  std::string img_pat_gray_left = path_image_gray_left_ + "/data/" + file_names_image_color_left_[file_index_];
  getline(ifs_image_gray_left_stamp, line);
  if(ifs_image_gray_left_stamp.eof())
    throw(e);
  utc2ts(line, timestamp);
  convert_image_to_msg(*image_message_gray_left, img_pat_gray_left, timestamp);

  auto image_message_gray_right = std::make_unique<sensor_msgs::msg::Image>();
  std::string img_pat_gray_right = path_image_gray_right_ + "/data/" + file_names_image_color_right_[file_index_];
  getline(ifs_image_gray_right_stamp, line);
  if(ifs_image_gray_right_stamp.eof())
    throw(e);
  utc2ts(line, timestamp);
  convert_image_to_msg(*image_message_gray_right, img_pat_gray_right, timestamp);

  auto image_message_color_left = std::make_unique<sensor_msgs::msg::Image>();
  std::string img_pat_color_left = path_image_color_right_ + "/data/" + file_names_image_color_left_[file_index_];
  getline(ifs_image_color_left_stamp, line);
  if(ifs_image_color_left_stamp.eof())
    throw(e);
  utc2ts(line, timestamp);
  convert_image_to_msg(*image_message_color_left, img_pat_color_left, timestamp);

  auto image_message_color_right = std::make_unique<sensor_msgs::msg::Image>();
  std::string img_pat_color_right = path_image_color_right_ + "/data/" + file_names_image_color_right_[file_index_];
  getline(ifs_image_color_right_stamp, line);
  if(ifs_image_color_right_stamp.eof())
    throw(e);
  utc2ts(line, timestamp);
  convert_image_to_msg(*image_message_color_right, img_pat_color_right, timestamp);
  // 02- KITTI IMAGE MESSAGES END //

  // 03- KITTI OXTS to IMU, NAV & MARKERARRAY MESSAGE START//
  std::string oxts_file_name = path_oxts_ + "/data/" + file_names_oxts_[file_index_];
  const std::string delimiter = " ";
  std::vector<std::string> oxts_parsed_array = parse_file_data_into_string_array(oxts_file_name, delimiter);
  getline(ifs_oxts_stamp, line);
  if(ifs_oxts_stamp.eof())
    throw(e);
  utc2ts(line, timestamp);
  // RCLCPP_INFO(this->get_logger(), "OxTs size: '%i'", oxts_parsed_array.size());

  auto nav_sat_fix_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
  prepare_navsatfix_msg(oxts_parsed_array , *nav_sat_fix_msg, timestamp);

  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
  prepare_odom_msg(oxts_parsed_array, *odom_msg, timestamp);

  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
  prepare_imu_msg(oxts_parsed_array , *imu_msg, timestamp);

  auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
  prepare_marker_array_msg(oxts_parsed_array , *marker_array_msg, timestamp);
  // 03- KITTI OXTS to IMU, NAV & MARKERARRAY MESSAGE END//

  publisher_point_cloud_->publish(point_cloud2_msg);
  publisher_image_gray_left_->publish(std::move(image_message_gray_left));
  publisher_image_gray_right_->publish(std::move(image_message_gray_right));
  publisher_image_color_left_->publish(std::move(image_message_color_left));
  publisher_image_color_right_->publish(std::move(image_message_color_right));

  publisher_imu_->publish(std::move(imu_msg));
  publisher_nav_sat_fix_->publish(std::move(nav_sat_fix_msg));
  publisher_marker_array_->publish(std::move(marker_array_msg));
  publisher_odom_->publish(std::move(odom_msg));

  odom2base_tf_bcaster->sendTransform(odom2base_tf_msg);

  imu2velo_tf_msg.header.stamp = odom2base_tf_msg.header.stamp;
  imu2velo_tf_bcaster->sendTransform(imu2velo_tf_msg);
  file_index_++;
}

void KittiPublishersNode::convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2 & msg, const std::string &filePath, const builtin_interfaces::msg::Time &stamp){
  pcl::PointCloud<pcl::PointXYZI> cloud;

  std::fstream input(filePath, std::ios::in | std::ios::binary);
  if(!input.good()){
    RCLCPP_ERROR(this->get_logger(), "Could not read Velodyne's point cloud at %s. Check your file path!", filePath.c_str());
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
  msg.header.frame_id = "velodyne";
  msg.header.stamp = stamp;
}

void KittiPublishersNode::init_file_path()
{
    path_point_cloud_ = "data/2011_09_26/2011_09_26_drive_0017_sync/velodyne_points/data/";
    path_image_gray_left_ = "data/2011_09_26/2011_09_26_drive_0017_sync/image_00/data/";
    path_image_gray_right_ = "data/2011_09_26/2011_09_26_drive_0017_sync/image_01/data/";
    path_image_color_left_ = "data/2011_09_26/2011_09_26_drive_0017_sync/image_02/data/";
    path_image_color_right_ = "data/2011_09_26/2011_09_26_drive_0017_sync/image_03/data/";
    path_oxts_ = "data/2011_09_26/2011_09_26_drive_0017_sync/oxts/data/";
}

std::string KittiPublishersNode::get_path(KittiPublishersNode::PublisherType publisher_type)
{
  std::string path;
  switch (publisher_type) {
    case KittiPublishersNode::PublisherType::POINT_CLOUD:
      path = path_point_cloud_;
      break;
    case KittiPublishersNode::PublisherType::IMAGE_LEFT_GRAY:
      path = path_image_gray_left_;
      break;
    case KittiPublishersNode::PublisherType::IMAGE_RIGHT_GRAY:
      path = path_image_gray_right_;
      break;
    case KittiPublishersNode::PublisherType::IMAGE_LEFT_COLOR:
      path = path_image_color_left_;
      break;
    case KittiPublishersNode::PublisherType::IMAGE_RIGHT_COLOR:
      path = path_image_color_right_;
      break;
    case KittiPublishersNode::PublisherType::ODOMETRY:
      path = path_oxts_;
      break;
    default:
      KittiExceptions e = PATH_NOT_FOUND;
      throw(e);
  }
  return path;
}

std::vector<std::string> KittiPublishersNode::get_filenames(PublisherType publisher_type)
{
  switch (publisher_type) {
    case KittiPublishersNode::PublisherType::POINT_CLOUD:
      return file_names_point_cloud_;
    case KittiPublishersNode::PublisherType::IMAGE_LEFT_GRAY:
      return file_names_image_gray_left_;
    case KittiPublishersNode::PublisherType::IMAGE_RIGHT_GRAY:
      return file_names_image_gray_right_;
    case KittiPublishersNode::PublisherType::IMAGE_LEFT_COLOR:
      return file_names_image_color_left_;
    case KittiPublishersNode::PublisherType::IMAGE_RIGHT_COLOR:
      return file_names_image_color_right_;
    case KittiPublishersNode::PublisherType::ODOMETRY:
      return file_names_oxts_;
    default:
      KittiExceptions e = PATH_NOT_FOUND;
      throw(e);
  }
}

void KittiPublishersNode::set_filenames(PublisherType publisher_type, std::vector<std::string> file_names)
{
  switch (publisher_type) {
    case KittiPublishersNode::PublisherType::POINT_CLOUD:
      file_names_point_cloud_= file_names;
      break;
    case KittiPublishersNode::PublisherType::IMAGE_LEFT_GRAY:
      file_names_image_gray_left_= file_names;
      break;
    case KittiPublishersNode::PublisherType::IMAGE_RIGHT_GRAY:
      file_names_image_gray_right_= file_names;
      break;
    case KittiPublishersNode::PublisherType::IMAGE_LEFT_COLOR:
      file_names_image_color_left_= file_names;
      break;
    case KittiPublishersNode::PublisherType::IMAGE_RIGHT_COLOR:
      file_names_image_color_right_ = file_names;
      break;
    case KittiPublishersNode::PublisherType::ODOMETRY:
      file_names_oxts_= file_names;
      break;
    default:
      KittiExceptions e = PATH_NOT_FOUND;
      throw(e);
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
      for (const auto & entry : std::filesystem::directory_iterator(get_path(type) + "/data")){
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

void KittiPublishersNode::prepare_odom_msg(std::vector<std::string> &oxts_tokenized_array, nav_msgs::msg::Odometry &msg, const builtin_interfaces::msg::Time &stamp)
{
  msg.header.frame_id = "odom";
  msg.header.stamp = stamp;

  msg.child_frame_id = "base_link";
  const double deg2rad = 0.017453292519943295;
  double lat = std::atof(oxts_tokenized_array[0].c_str()) * deg2rad;
  double lon = std::atof(oxts_tokenized_array[1].c_str()) * deg2rad;
  double alt = std::atof(oxts_tokenized_array[2].c_str());
  double east=0.0, north=0.0, up=0.0;
  if(!odom_inited_) {
    geodetic2ecef(lat, lon, alt, ecef_x0, ecef_y0, ecef_h0);
    lat0 = lat, lon0 = lon;
    odom_inited_ = true;
  } else {
    double ecef_x, ecef_y, ecef_h;
    geodetic2ecef(lat, lon, alt, ecef_x, ecef_y, ecef_h);
    uvw2enu(ecef_x - ecef_x0, ecef_y - ecef_y0, ecef_h - ecef_h0, east, north, up);
  }
  msg.pose.pose.position.x = east;
  msg.pose.pose.position.y = north;
  msg.pose.pose.position.z = up;

  tf2::Quaternion q;
  q.setRPY(std::atof(oxts_tokenized_array[3].c_str()),
            std::atof(oxts_tokenized_array[4].c_str()),
            std::atof(oxts_tokenized_array[5].c_str()));
  q.normalize();
  msg.pose.pose.orientation.x = q.getX();
  msg.pose.pose.orientation.y = q.getY();
  msg.pose.pose.orientation.z = q.getZ();
  msg.pose.pose.orientation.w = q.getW();

  msg.twist.twist.linear.x = std::atof(oxts_tokenized_array[7].c_str());
  msg.twist.twist.linear.y = std::atof(oxts_tokenized_array[6].c_str());
  msg.twist.twist.linear.z = std::atof(oxts_tokenized_array[10].c_str());
  msg.twist.twist.angular.x = std::atof(oxts_tokenized_array[20].c_str());
  msg.twist.twist.angular.y = std::atof(oxts_tokenized_array[21].c_str());
  msg.twist.twist.angular.z = std::atof(oxts_tokenized_array[22].c_str());

  odom2base_tf_msg.header.stamp = stamp;
  odom2base_tf_msg.header.frame_id = "odom";
  odom2base_tf_msg.child_frame_id = "base_link";
  odom2base_tf_msg.transform.translation.x = msg.pose.pose.position.x;
  odom2base_tf_msg.transform.translation.y = msg.pose.pose.position.y;
  odom2base_tf_msg.transform.translation.z = msg.pose.pose.position.z;
  odom2base_tf_msg.transform.rotation.x = msg.pose.pose.orientation.x;
  odom2base_tf_msg.transform.rotation.y = msg.pose.pose.orientation.y;
  odom2base_tf_msg.transform.rotation.z = msg.pose.pose.orientation.z;
  odom2base_tf_msg.transform.rotation.w = msg.pose.pose.orientation.w;
}

void KittiPublishersNode::geodetic2ecef(double lat, double lon, double h, double &x, double &y, double &z)
{
  double N = semimajor_axis * semimajor_axis / std::sqrt(semimajor_axis * semimajor_axis * std::cos(lat) * std::cos(lat)
                + semiminor_axis * semiminor_axis *std::sin(lat) * std::sin(lat));
  x = (N + h) * std::cos(lat) * std::cos(lon);
  y = (N + h) * std::cos(lat) * std::sin(lon);
  z = (N * (semiminor_axis / semimajor_axis) * (semiminor_axis / semimajor_axis) + h) * std::sin(lat);
}

void KittiPublishersNode::uvw2enu(double u, double v, double w, double &east, double &north, double &up)
{
  double t = std::cos(lon0) * u + std::sin(lon0) * v;
  east = -std::sin(lon0) * u + std::cos(lon0) * v;
  up = std::cos(lat0) * t + std::sin(lat0) * w;
  north = -std::sin(lat0) * t + std::cos(lat0) * w;
}


void KittiPublishersNode::prepare_navsatfix_msg(std::vector<std::string> &oxts_tokenized_array, sensor_msgs::msg::NavSatFix &msg, const builtin_interfaces::msg::Time &stamp)
{
  msg.header.frame_id = "base_link";
  msg.header.stamp = stamp;

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
void KittiPublishersNode::prepare_marker_array_msg(std::vector<std::string> &oxts_tokenized_array, visualization_msgs::msg::MarkerArray &msg, const builtin_interfaces::msg::Time &stamp)
{
  const double lat =  std::stod(oxts_tokenized_array[0]);
  const double lon =  std::stod(oxts_tokenized_array[1]);

  std::array<double, 2> WGS84Reference{lat, lon};
  std::array<double, 2> WGS84Position{lat, lon};
  std::array<double, 2> result{wgs84::toCartesian(WGS84Reference, WGS84Position)};

  visualization_msgs::msg::Marker RTK_MARKER;

  static int gps_track = 1;
  RTK_MARKER.header.frame_id = "base_link";
  RTK_MARKER.header.stamp = stamp;
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
void KittiPublishersNode::prepare_imu_msg(std::vector<std::string> &oxts_tokenized_array, sensor_msgs::msg::Imu &msg, const builtin_interfaces::msg::Time &stamp){
  msg.header.frame_id = "base_link";
  msg.header.stamp = stamp;

  //    - ax:      acceleration in x, i.e. in direction of vehicle front (m/s^2)
  //    - ay:      acceleration in y, i.e. in direction of vehicle left (m/s^2)
  //    - az:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
  msg.linear_acceleration.x = std::atof(oxts_tokenized_array[11].c_str());
  msg.linear_acceleration.y = std::atof(oxts_tokenized_array[12].c_str());
  msg.linear_acceleration.z = std::atof(oxts_tokenized_array[13].c_str());

  //    - vf:      forward velocity, i.e. parallel to earth-surface (m/s)
  //    - vl:      leftward velocity, i.e. parallel to earth-surface (m/s)
  //    - vu:      upward velocity, i.e. perpendicular to earth-surface (m/s)
  msg.angular_velocity.x = std::atof(oxts_tokenized_array[17].c_str());
  msg.angular_velocity.y = std::atof(oxts_tokenized_array[18].c_str());
  msg.angular_velocity.z = std::atof(oxts_tokenized_array[19].c_str());

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
void KittiPublishersNode::convert_image_to_msg(sensor_msgs::msg::Image & msg, const std::string path, const builtin_interfaces::msg::Time &stamp)
{
  Mat frame;
  frame = imread(path);
  if (frame.empty())                      // Check for invalid input
  {
    RCLCPP_ERROR(this->get_logger(), "Image does not exist at path %s. Check your files path!", path.c_str());
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
  msg.header.frame_id = "camera";
  msg.header.stamp = stamp;
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
    RCLCPP_ERROR(this->get_logger(), "Could not read OXTS data at path %s. Check your file path!", file_name.c_str());
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

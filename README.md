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

## **Requirements**

- Ubuntu 22.04+ (tested on Ubuntu 24.04)
- ROS2 Jazzy Jalisco
- pcl (Point Cloud Library)
- OpenCV 4.x
- Rviz2
- rqt (optional)

---
## __Setup__

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

2. **Download KITTI Dataset:**
   - Download the **Synced+Rectified Data** version from [KITTI Raw Data](http://www.cvlibs.net/datasets/kitti/raw_data.php)
   - Extract the dataset to your desired location (e.g., `/path/to/kitti/2011_09_26/2011_09_26_drive_0014_sync/`)

3. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros2_kitti_publishers
   source install/setup.bash
   ```

## __Usage__

### Using Launch File (Recommended)

The easiest way to run the KITTI publishers is using the launch file, which automatically starts RViz2:

```bash
ros2 launch ros2_kitti_publishers kitti_publishers.launch.py \
  dataset_base_path:=/path/to/kitti/2011_09_26/2011_09_26_drive_0014_sync/
```

**Launch File Parameters:**
- `dataset_base_path` (REQUIRED): Base path to KITTI dataset directory
- `frame_id` (default: `base_link`): ROS2 frame ID for all published messages
- `publish_rate` (default: `10.0`): Publishing rate in Hz
- `use_rviz` (default: `true`): Whether to launch RViz2 automatically
- `rviz_config`: Path to custom RViz2 configuration file (default: package config)

**Examples:**
```bash
# Basic usage with RViz2
ros2 launch ros2_kitti_publishers kitti_publishers.launch.py \
  dataset_base_path:=/home/user/kitti/2011_09_26/2011_09_26_drive_0014_sync/

# Without RViz2
ros2 launch ros2_kitti_publishers kitti_publishers.launch.py \
  dataset_base_path:=/path/to/dataset/ \
  use_rviz:=false

# Custom frame ID and publish rate
ros2 launch ros2_kitti_publishers kitti_publishers.launch.py \
  dataset_base_path:=/path/to/dataset/ \
  frame_id:=kitti_frame \
  publish_rate:=5.0
```

### Using ros2 run

You can also run the node directly with parameters:

```bash
ros2 run ros2_kitti_publishers kitti_publishers \
  --ros-args -p dataset_base_path:=/path/to/kitti/2011_09_26/2011_09_26_drive_0014_sync/ \
             -p frame_id:=base_link \
             -p publish_rate:=10.0
```

**Note:** The `dataset_base_path` parameter is **required** and must be provided.

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
ros2_kitti_publishers/
├ include/
│   └── ros2_kitti_publishers/
│       ├── kitti_publishers_node.hpp
│       ├── visibility.h
│       └── WGS84toCartesian.hpp
├ src/
│   ├── node/
│   │   └── kitti_publishers_node.cpp
│   └── kitti_publishers_main.cpp
├ launch/
│   └── kitti_publishers.launch.py
├ config/
│   ├── kitti_publishers_params.yaml
│   └── kitti_publishers.rviz
├ package.xml
├ CMakeLists.txt
├ README.md
└ LICENSE
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

// Timer period calculated from publish_rate parameter (default: 10 Hz)
auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
timer_ = create_wall_timer(timer_period, std::bind(&KittiPublishersNode::on_timer_callback, this));
```

* ##  The main operation/pipeline was implemented inside the callback function of the timer object.

```cpp
void KittiPublishersNode::on_timer_callback()
{
    // Load real timestamps from KITTI dataset files
    rclcpp::Time point_cloud_timestamp = timestamps_point_cloud_[current_index];
    rclcpp::Time image_timestamp = timestamps_image_gray_left_[current_index];
    rclcpp::Time oxts_timestamp = timestamps_oxts_[current_index];
    
    // Read files in parallel using std::async
    auto point_cloud_future = std::async(...);
    auto image_future = std::async(...);
    
    // Convert data to ROS2 messages with real KITTI timestamps
    convert_pcl_to_pointcloud2(msg, current_index);
    msg.header.stamp = point_cloud_timestamp;  // Real timestamp from dataset
    
    convert_image_to_msg(*image_msg, path);
    image_msg->header.stamp = image_timestamp;  // Real timestamp from dataset
    
    prepare_navsatfix_msg(oxts_parsed_array, *nav_sat_fix_msg);
    nav_sat_fix_msg->header.stamp = oxts_timestamp;  // Real timestamp from dataset
    
    prepare_imu_msg(oxts_parsed_array, *imu_msg);
    prepare_marker_array_msg(oxts_parsed_array, *marker_array_msg);
    
    // Publish all messages with their respective timestamps
    publisher_point_cloud_->publish(msg);
    publisher_image_gray_left_->publish(std::move(image_msg));
    // ... publish other messages
}
```

**Key Features:**
- **Real Timestamps**: Each message uses its corresponding timestamp from KITTI dataset files
- **Parallel Reading**: Files are read asynchronously for better performance
- **Synchronization**: All messages maintain accurate temporal relationships from the original dataset

##  Visualization

### RViz2

RViz2 is automatically launched when using the launch file (default behavior). The configuration file is located at `config/kitti_publishers.rviz` and includes:
- Point cloud visualization
- Image displays for all camera feeds
- GPS/IMU marker visualization
- TF frame visualization

To use a custom RViz configuration:
```bash
ros2 launch ros2_kitti_publishers kitti_publishers.launch.py \
  dataset_base_path:=/path/to/dataset/ \
  rviz_config:=/path/to/custom.rviz
```

### Published Topics

The node publishes the following topics:
- `kitti/point_cloud` (sensor_msgs/PointCloud2): Velodyne point cloud data
- `kitti/image/gray/left` (sensor_msgs/Image): Left grayscale camera
- `kitti/image/gray/right` (sensor_msgs/Image): Right grayscale camera
- `kitti/image/color/left` (sensor_msgs/Image): Left color camera
- `kitti/image/color/right` (sensor_msgs/Image): Right color camera
- `kitti/imu` (sensor_msgs/Imu): IMU data from OXTS
- `kitti/nav_sat_fix` (sensor_msgs/NavSatFix): GPS data from OXTS
- `kitti/marker_array` (visualization_msgs/MarkerArray): GPS trajectory markers

## __Configuration__

### ROS2 Parameters

The node supports the following ROS2 parameters (configurable via launch file or command line):

- `dataset_base_path` (string, **REQUIRED**): Base path to KITTI dataset directory
- `frame_id` (string, default: `"base_link"`): ROS2 frame ID for all published messages
- `publish_rate` (double, default: `10.0`): Publishing rate in Hz

### Configuration Files

- **Parameters**: `config/kitti_publishers_params.yaml` - ROS2 parameter configuration
- **RViz Config**: `config/kitti_publishers.rviz` - RViz2 visualization configuration

## __Timestamp Synchronization__

### Real KITTI Timestamps

The node now uses **real timestamps from KITTI dataset files** instead of system time. This ensures accurate temporal synchronization between all sensor data types.

**How it works:**
1. **Timestamp Loading**: On initialization, the node reads `timestamps.txt` files from each sensor directory:
   - `velodyne_points/timestamps.txt`
   - `image_00/timestamps.txt` (left grayscale)
   - `image_01/timestamps.txt` (right grayscale)
   - `image_02/timestamps.txt` (left color)
   - `image_03/timestamps.txt` (right color)
   - `oxts/timestamps.txt` (GPS/IMU)

2. **Timestamp Parsing**: KITTI timestamp format (`2011-09-26 13:11:15.406628381`) is parsed and converted to ROS2 `rclcpp::Time` format.

3. **Message Timestamps**: Each published message uses its corresponding real timestamp from the dataset:
   - Point cloud messages use timestamps from `velodyne_points/timestamps.txt`
   - Image messages use timestamps from their respective `image_XX/timestamps.txt` files
   - IMU, NavSatFix, and MarkerArray messages use timestamps from `oxts/timestamps.txt`

4. **Dataset Start Time (t0)**: The earliest timestamp across all sensors is calculated and stored as the dataset reference time.

**Benefits:**
- ✅ Accurate temporal relationships between sensor data
- ✅ Proper synchronization for sensor fusion algorithms
- ✅ Compatible with both sync and async KITTI datasets
- ✅ Maintains original dataset timing characteristics

**Example:**
```
[INFO] [kitti_publishers]: Loaded 314 point cloud timestamps
[INFO] [kitti_publishers]: Loaded 314 image gray left timestamps
[INFO] [kitti_publishers]: Dataset start time (t0): 1317042675.388194561
```

## __Recent Updates__

### Timestamp Synchronization (Latest)
- **Real KITTI timestamps**: All messages now use timestamps from KITTI dataset files
- **Timestamp parsing**: Automatic parsing of KITTI timestamp format to ROS2 Time
- **Per-sensor timestamps**: Each sensor type uses its own timestamp from the dataset
- **Dataset reference time**: Calculates and stores the earliest timestamp (t0) as reference
- **Sync/Async support**: Works with both synchronized and asynchronous KITTI datasets

### ROS2 Jazzy Compatibility
- Updated to work with ROS2 Jazzy Jalisco
- Improved error handling and robustness
- Added comprehensive bounds checking
- Fixed segmentation faults in point cloud reading

### Configuration Management
- Moved from hard-coded paths to ROS2 parameters
- Added launch file support with automatic RViz2 integration
- Configuration files organized in `config/` directory

### Improvements
- Better error messages and logging
- Windows Zone.Identifier file filtering
- Improved OXTS data parsing
- Performance optimizations with parallel file reading

## __Potential Improvements__

This project is continuously being improved. Planned enhancements include:
- **SOLID principles refactoring** (see `REFACTORING_PLAN.md` for detailed plan)
  - Single Responsibility Principle: Separate file I/O, conversion, and publishing concerns
  - Open/Closed Principle: Use interfaces and factory patterns for extensibility
  - Dependency Inversion: Abstract interfaces and dependency injection
- Camera calibration service implementation
- Support for async KITTI datasets
- Unit tests and integration tests
- Better documentation

**Next Steps:** See `REFACTORING_PLAN.md` for the detailed refactoring roadmap.

Contributions and suggestions are welcome! You can:
- Open an issue for bugs or feature requests
- Submit pull requests
- Contact: umtclskn@gmail.com

# References
1. http://www.cvlibs.net/datasets/kitti/raw_data.php
2. https://github.com/iralabdisco/kitti_player
3. https://github.com/chrberger/WGS84toCartesian
4. https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp#L278
5. https://github.com/yanii/kitti-pcl
6. https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/

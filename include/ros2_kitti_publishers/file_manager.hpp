#ifndef ROS2_KITTI_PUBLISHERS__FILE_MANAGER_HPP_
#define ROS2_KITTI_PUBLISHERS__FILE_MANAGER_HPP_

#include <string>
#include <vector>
#include <filesystem>

namespace ros2_kitti_publishers
{

/**
 * @brief Manages file paths and file operations for KITTI dataset
 * 
 * This class is responsible for:
 * - Initializing dataset paths
 * - Getting file lists from directories
 * - Parsing file contents into arrays
 */
class FileManager
{
public:
  /**
   * @brief Structure to hold all dataset directory paths
   */
  struct DatasetPaths
  {
    std::string point_cloud;
    std::string image_gray_left;
    std::string image_gray_right;
    std::string image_color_left;
    std::string image_color_right;
    std::string oxts;
    
    // Timestamp file paths
    std::string timestamps_point_cloud;
    std::string timestamps_image_gray_left;
    std::string timestamps_image_gray_right;
    std::string timestamps_image_color_left;
    std::string timestamps_image_color_right;
    std::string timestamps_oxts;
  };

  /**
   * @brief Initialize all dataset paths from base path
   * @param base_path Base path to KITTI dataset directory
   * @return DatasetPaths structure with all paths initialized
   */
  DatasetPaths initializePaths(const std::string& base_path);

  /**
   * @brief Get sorted list of files from a directory
   * @param directory_path Path to directory
   * @return Sorted vector of filenames (excluding system files)
   */
  std::vector<std::string> getFileList(const std::string& directory_path);

  /**
   * @brief Parse file content into string array using delimiter
   * @param file_path Path to file
   * @param delimiter Delimiter character(s) to split on
   * @return Vector of tokens (trimmed and filtered)
   */
  std::vector<std::string> parseFileIntoArray(
    const std::string& file_path,
    const std::string& delimiter);

private:
  /**
   * @brief Normalize path by ensuring it ends with '/'
   * @param path Input path
   * @return Normalized path
   */
  std::string normalizePath(const std::string& path);

  /**
   * @brief Check if filename should be filtered (system files, etc.)
   * @param filename Filename to check
   * @return true if file should be filtered out
   */
  bool shouldFilterFile(const std::string& filename);
};

}  // namespace ros2_kitti_publishers

#endif  // ROS2_KITTI_PUBLISHERS__FILE_MANAGER_HPP_


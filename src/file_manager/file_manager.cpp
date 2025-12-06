#include "ros2_kitti_publishers/file_manager.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

namespace ros2_kitti_publishers
{

FileManager::DatasetPaths FileManager::initializePaths(const std::string& base_path)
{
  DatasetPaths paths;
  std::string normalized_path = normalizePath(base_path);

  // Data directories
  paths.point_cloud = normalized_path + "velodyne_points/data/";
  paths.image_gray_left = normalized_path + "image_00/data/";
  paths.image_gray_right = normalized_path + "image_01/data/";
  paths.image_color_left = normalized_path + "image_02/data/";
  paths.image_color_right = normalized_path + "image_03/data/";
  paths.oxts = normalized_path + "oxts/data/";

  // Timestamp files
  paths.timestamps_point_cloud = normalized_path + "velodyne_points/timestamps.txt";
  paths.timestamps_image_gray_left = normalized_path + "image_00/timestamps.txt";
  paths.timestamps_image_gray_right = normalized_path + "image_01/timestamps.txt";
  paths.timestamps_image_color_left = normalized_path + "image_02/timestamps.txt";
  paths.timestamps_image_color_right = normalized_path + "image_03/timestamps.txt";
  paths.timestamps_oxts = normalized_path + "oxts/timestamps.txt";

  return paths;
}

std::vector<std::string> FileManager::getFileList(const std::string& directory_path)
{
  std::vector<std::string> file_names;

  try {
    if (std::filesystem::exists(directory_path) && std::filesystem::is_directory(directory_path)) {
      for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
        if (entry.is_regular_file()) {
          std::string filename = entry.path().filename().string();
          if (!shouldFilterFile(filename)) {
            file_names.push_back(filename);
          }
        }
      }

      // Sort file names
      std::sort(file_names.begin(), file_names.end(),
                [](const auto& lhs, const auto& rhs) {
                  return lhs < rhs;
                });
    }
  } catch (const std::filesystem::filesystem_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("file_manager"),
                 "Filesystem error while reading directory %s: %s",
                 directory_path.c_str(), e.what());
  }

  return file_names;
}

std::vector<std::string> FileManager::parseFileIntoArray(
  const std::string& file_path,
  const std::string& delimiter)
{
  std::vector<std::string> tokens;
  std::ifstream file(file_path);

  if (!file.is_open()) {
    RCLCPP_WARN(rclcpp::get_logger("file_manager"),
                "Could not open file: %s", file_path.c_str());
    return tokens;
  }

  std::string line;
  while (std::getline(file, line)) {
    // Trim trailing newline/whitespace
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r' || line.back() == ' ')) {
      line.pop_back();
    }

    if (line.empty()) {
      continue;
    }

    // Split by delimiter
    size_t pos = 0;
    std::string token;
    while ((pos = line.find(delimiter)) != std::string::npos) {
      token = line.substr(0, pos);
      // Trim whitespace
      token.erase(0, token.find_first_not_of(" \t"));
      token.erase(token.find_last_not_of(" \t") + 1);
      if (!token.empty()) {
        tokens.push_back(token);
      }
      line.erase(0, pos + delimiter.length());
    }

    // Add remaining part
    line.erase(0, line.find_first_not_of(" \t"));
    line.erase(line.find_last_not_of(" \t") + 1);
    if (!line.empty()) {
      tokens.push_back(line);
    }
  }

  file.close();
  return tokens;
}

std::string FileManager::normalizePath(const std::string& path)
{
  std::string normalized = path;
  if (!normalized.empty() && normalized.back() != '/') {
    normalized += "/";
  }
  return normalized;
}

bool FileManager::shouldFilterFile(const std::string& filename)
{
  // Filter out Windows Zone.Identifier files and hidden files
  return filename.find("Zone.Identifier") != std::string::npos || filename[0] == '.';
}

}  // namespace ros2_kitti_publishers


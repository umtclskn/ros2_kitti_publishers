// Copyright 2020 Umut Çalışkan
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

#include <memory>
#include "ros2_kitti_publishers/kitti_publishers_node.hpp"


#include "rclcpp/rclcpp.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  try {
    rclcpp::spin(std::make_shared<KittiPublishersNode>());
  } catch (const KittiExceptions &e) {
    switch (e) {
    case KittiExceptions::PATH_NOT_FOUND:
      std::cerr << "Path not found!\n";
      break;
    case KittiExceptions::END_OF_FILE:
      std::cerr << "Read end of file, process done.\n";
      break;
    default:
      std::cerr << "Unknown error!\n";
    }
  }
  rclcpp::shutdown();

  return 0;
}
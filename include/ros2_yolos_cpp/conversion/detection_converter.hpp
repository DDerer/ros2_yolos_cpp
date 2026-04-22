// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__CONVERSION__DETECTION_CONVERTER_HPP_
#define ROS2_YOLOS_CPP__CONVERSION__DETECTION_CONVERTER_HPP_

#include <vector>
#include <string>

#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <std_msgs/msg/header.hpp>

#include "ros2_yolos_cpp/adapters/yolos_adapter_base.hpp"
#include "ros2_yolos_cpp/visibility_control.hpp"

namespace ros2_yolos_cpp {
namespace conversion {

/// @brief Convert DetectionResult to vision_msgs::msg::Detection2D
ROS2_YOLOS_CPP_PUBLIC
vision_msgs::msg::Detection2D toDetection2D(
  const DetectionResult& det,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height);

/// @brief Convert vector of DetectionResults to Detection2DArray
ROS2_YOLOS_CPP_PUBLIC
vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<DetectionResult>& detections,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height);

}  // namespace conversion
}  // namespace ros2_yolos_cpp

#endif  // ROS2_YOLOS_CPP__CONVERSION__DETECTION_CONVERTER_HPP_

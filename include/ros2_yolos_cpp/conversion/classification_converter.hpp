// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__CONVERSION__CLASSIFICATION_CONVERTER_HPP_
#define ROS2_YOLOS_CPP__CONVERSION__CLASSIFICATION_CONVERTER_HPP_

#include <vision_msgs/msg/classification.hpp>
#include <std_msgs/msg/header.hpp>

#include "ros2_yolos_cpp/adapters/yolos_adapter_base.hpp"
#include "ros2_yolos_cpp/visibility_control.hpp"

namespace ros2_yolos_cpp {
namespace conversion {

/// @brief Convert ClassificationResult to vision_msgs::msg::Classification
ROS2_YOLOS_CPP_PUBLIC
vision_msgs::msg::Classification toClassification(
  const ClassificationResult& result,
  const std_msgs::msg::Header& header);

}  // namespace conversion
}  // namespace ros2_yolos_cpp

#endif  // ROS2_YOLOS_CPP__CONVERSION__CLASSIFICATION_CONVERTER_HPP_

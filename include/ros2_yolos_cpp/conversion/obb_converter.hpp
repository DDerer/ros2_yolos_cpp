// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__CONVERSION__OBB_CONVERTER_HPP_
#define ROS2_YOLOS_CPP__CONVERSION__OBB_CONVERTER_HPP_

#include <vector>
#include <std_msgs/msg/header.hpp>

#include "ros2_yolos_cpp/adapters/yolos_adapter_base.hpp"
#include "ros2_yolos_cpp/visibility_control.hpp"

// Forward declare custom message
#include "ros2_yolos_cpp/msg/obb_detection2_d_array.hpp"

namespace ros2_yolos_cpp {
namespace conversion {

/// @brief Convert OBBResults to custom OBBDetection2DArray message
ROS2_YOLOS_CPP_PUBLIC
ros2_yolos_cpp::msg::OBBDetection2DArray toOBBDetection2DArray(
  const std::vector<OBBResult>& detections,
  const std_msgs::msg::Header& header);

}  // namespace conversion
}  // namespace ros2_yolos_cpp

#endif  // ROS2_YOLOS_CPP__CONVERSION__OBB_CONVERTER_HPP_

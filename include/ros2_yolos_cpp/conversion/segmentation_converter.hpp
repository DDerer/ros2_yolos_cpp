// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__CONVERSION__SEGMENTATION_CONVERTER_HPP_
#define ROS2_YOLOS_CPP__CONVERSION__SEGMENTATION_CONVERTER_HPP_

#include <vector>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "ros2_yolos_cpp/adapters/yolos_adapter_base.hpp"
#include "ros2_yolos_cpp/visibility_control.hpp"

namespace ros2_yolos_cpp {
namespace conversion {

/// @brief Convert SegmentationResults to Detection2DArray (boxes only)
ROS2_YOLOS_CPP_PUBLIC
vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<SegmentationResult>& segmentations,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height);

/// @brief Convert mask to sensor_msgs::msg::Image
ROS2_YOLOS_CPP_PUBLIC
sensor_msgs::msg::Image toMaskImage(
  const cv::Mat& mask,
  const std_msgs::msg::Header& header);

/// @brief Combine all masks into single multi-class mask image
ROS2_YOLOS_CPP_PUBLIC
sensor_msgs::msg::Image toCombinedMaskImage(
  const std::vector<SegmentationResult>& segmentations,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height);

}  // namespace conversion
}  // namespace ros2_yolos_cpp

#endif  // ROS2_YOLOS_CPP__CONVERSION__SEGMENTATION_CONVERTER_HPP_

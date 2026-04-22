// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#include "ros2_yolos_cpp/conversion/detection_converter.hpp"

#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace ros2_yolos_cpp {
namespace conversion {

vision_msgs::msg::Detection2D toDetection2D(
  const DetectionResult& det,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height)
{
  vision_msgs::msg::Detection2D msg;
  msg.header = header;

  // Convert from top-left corner (x, y, w, h) to center-based
  // BoundingBox2D uses center coordinates
  msg.bbox.center.position.x = det.bbox.x + det.bbox.width / 2.0;
  msg.bbox.center.position.y = det.bbox.y + det.bbox.height / 2.0;
  msg.bbox.size_x = det.bbox.width;
  msg.bbox.size_y = det.bbox.height;

  // Add hypothesis (class + score)
  vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  hypothesis.hypothesis.class_id = det.class_name.empty() ? 
    std::to_string(det.class_id) : det.class_name;
  hypothesis.hypothesis.score = det.confidence;
  msg.results.push_back(hypothesis);

  // Optional: Set source image size in id field for downstream use
  msg.id = "det_" + std::to_string(det.class_id);

  // Suppress unused parameter warnings
  (void)image_width;
  (void)image_height;

  return msg;
}

vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<DetectionResult>& detections,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height)
{
  vision_msgs::msg::Detection2DArray msg;
  msg.header = header;
  msg.detections.reserve(detections.size());

  for (const auto& det : detections) {
    msg.detections.push_back(
      toDetection2D(det, header, image_width, image_height)
    );
  }

  return msg;
}

}  // namespace conversion
}  // namespace ros2_yolos_cpp

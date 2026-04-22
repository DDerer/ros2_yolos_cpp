// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#include "ros2_yolos_cpp/conversion/obb_converter.hpp"
#include "ros2_yolos_cpp/msg/obb_detection2_d.hpp"
#include "ros2_yolos_cpp/msg/oriented_bounding_box2_d.hpp"

namespace ros2_yolos_cpp {
namespace conversion {

ros2_yolos_cpp::msg::OBBDetection2DArray toOBBDetection2DArray(
  const std::vector<OBBResult>& detections,
  const std_msgs::msg::Header& header)
{
  ros2_yolos_cpp::msg::OBBDetection2DArray msg;
  msg.header = header;
  msg.detections.reserve(detections.size());

  for (const auto& det : detections) {
    ros2_yolos_cpp::msg::OBBDetection2D obb_msg;
    obb_msg.header = header;
    
    // Oriented bounding box
    obb_msg.bbox.center.x = det.bbox.center_x;
    obb_msg.bbox.center.y = det.bbox.center_y;
    obb_msg.bbox.center.z = 0.0;  // 2D detection
    obb_msg.bbox.size_x = det.bbox.width;
    obb_msg.bbox.size_y = det.bbox.height;
    obb_msg.bbox.theta = det.bbox.angle;
    
    obb_msg.class_id = det.class_id;
    obb_msg.class_name = det.class_name;
    obb_msg.score = det.confidence;

    msg.detections.push_back(obb_msg);
  }

  return msg;
}

}  // namespace conversion
}  // namespace ros2_yolos_cpp

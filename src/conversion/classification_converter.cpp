// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#include "ros2_yolos_cpp/conversion/classification_converter.hpp"
#include <vision_msgs/msg/object_hypothesis.hpp>

namespace ros2_yolos_cpp {
namespace conversion {

vision_msgs::msg::Classification toClassification(
  const ClassificationResult& result,
  const std_msgs::msg::Header& header)
{
  vision_msgs::msg::Classification msg;
  msg.header = header;

  vision_msgs::msg::ObjectHypothesis hypothesis;
  hypothesis.class_id = result.class_name.empty() ? 
    std::to_string(result.class_id) : result.class_name;
  hypothesis.score = result.confidence;
  msg.results.push_back(hypothesis);

  return msg;
}

}  // namespace conversion
}  // namespace ros2_yolos_cpp

// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__NODES__SEGMENTOR_NODE_HPP_
#define ROS2_YOLOS_CPP__NODES__SEGMENTOR_NODE_HPP_

#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include "ros2_yolos_cpp/visibility_control.hpp"
#include "ros2_yolos_cpp/adapters/segmentor_adapter.hpp"

namespace ros2_yolos_cpp {

class ROS2_YOLOS_CPP_PUBLIC YolosSegmentorNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit YolosSegmentorNode(const rclcpp::NodeOptions& options);
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& s) override { return on_cleanup(s); }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void declareParameters();
  YolosConfig loadConfig();
  std::unique_ptr<ISegmentorAdapter> segmentor_;
  rclcpp::CallbackGroup::SharedPtr cb_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  float conf_, nms_;
  bool debug_;
};

}  // namespace ros2_yolos_cpp
#endif

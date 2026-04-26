// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__NODES__DETECTOR_NODE_HPP_
#define ROS2_YOLOS_CPP__NODES__DETECTOR_NODE_HPP_

#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <string>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "ros2_yolos_cpp/adapters/detector_adapter.hpp"
#include "ros2_yolos_cpp/srv/detect_image.hpp"
#include "ros2_yolos_cpp/visibility_control.hpp"

namespace ros2_yolos_cpp {

class ROS2_YOLOS_CPP_PUBLIC YolosDetectorNode : public rclcpp_lifecycle::LifecycleNode {
   public:
    explicit YolosDetectorNode(const rclcpp::NodeOptions& options);
    ~YolosDetectorNode() override = default;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

   private:
    void declareParameters();
    YolosConfig loadConfig();
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void detectServiceCallback(const std::shared_ptr<srv::DetectImage::Request> request,
                               std::shared_ptr<srv::DetectImage::Response> response);
    vision_msgs::msg::Detection2DArray processFrame(const cv::Mat& frame, const std_msgs::msg::Header& header);

    std::unique_ptr<IDetectorAdapter> detector_;
    rclcpp::CallbackGroup::SharedPtr inference_cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Service<srv::DetectImage>::SharedPtr detect_service_;
    std::mutex latest_frame_mutex_;
    cv::Mat latest_frame_;
    std_msgs::msg::Header latest_header_;
    bool has_latest_frame_{false};

    rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32MultiArray>::SharedPtr offset_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr offset_m_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr distance_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;

    std::string model_path_, labels_path_, yolo_version_;
    bool use_gpu_{false}, publish_timing_{false};
    float conf_threshold_{0.4f}, nms_threshold_{0.45f};
    double camera_fx_{800.0};
    double camera_fy_{800.0};
    double camera_cx_{320.0};
    double camera_cy_{240.0};
    double target_plane_distance_m_{2.0};
};

}  // namespace ros2_yolos_cpp

#endif  // ROS2_YOLOS_CPP__NODES__DETECTOR_NODE_HPP_

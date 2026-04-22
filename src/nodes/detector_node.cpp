// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#include "ros2_yolos_cpp/nodes/detector_node.hpp"

#include "ros2_yolos_cpp/conversion/detection_converter.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <chrono>
#include <cmath>

namespace ros2_yolos_cpp {

YolosDetectorNode::YolosDetectorNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("yolos_detector", options) {
    RCLCPP_INFO(get_logger(), "YolosDetectorNode created");
    declareParameters();
}

void YolosDetectorNode::declareParameters() {
    declare_parameter("model_path", rclcpp::PARAMETER_STRING);
    declare_parameter("labels_path", rclcpp::PARAMETER_STRING);
    declare_parameter("use_gpu", false);
    declare_parameter("conf_threshold", 0.4);
    declare_parameter("nms_threshold", 0.45);
    declare_parameter("yolo_version", "auto");
    declare_parameter("publish_timing", false);
    declare_parameter("camera_fx", 800.0);
    declare_parameter("camera_fy", 800.0);
    declare_parameter("camera_cx", 320.0);
    declare_parameter("camera_cy", 240.0);
    declare_parameter("target_plane_distance_m", 2.0);
}

YolosConfig YolosDetectorNode::loadConfig() {
    YolosConfig config;
    config.model_path = get_parameter("model_path").as_string();
    config.labels_path = get_parameter("labels_path").as_string();
    config.use_gpu = get_parameter("use_gpu").as_bool();
    config.conf_threshold = static_cast<float>(get_parameter("conf_threshold").as_double());
    config.nms_threshold = static_cast<float>(get_parameter("nms_threshold").as_double());
    config.yolo_version = get_parameter("yolo_version").as_string();

    model_path_ = config.model_path;
    labels_path_ = config.labels_path;
    use_gpu_ = config.use_gpu;
    conf_threshold_ = config.conf_threshold;
    nms_threshold_ = config.nms_threshold;
    yolo_version_ = config.yolo_version;
    publish_timing_ = get_parameter("publish_timing").as_bool();
    camera_fx_ = get_parameter("camera_fx").as_double();
    camera_fy_ = get_parameter("camera_fy").as_double();
    camera_cx_ = get_parameter("camera_cx").as_double();
    camera_cy_ = get_parameter("camera_cy").as_double();
    target_plane_distance_m_ = get_parameter("target_plane_distance_m").as_double();

    return config;
}

YolosDetectorNode::CallbackReturn YolosDetectorNode::on_configure(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "Configuring...");
    try {
        auto config = loadConfig();
        if (config.model_path.empty() || config.labels_path.empty()) {
            RCLCPP_ERROR(get_logger(), "model_path and labels_path are required");
            return CallbackReturn::FAILURE;
        }
        detector_ = createDetectorAdapter();
        if (!detector_->initialize(config)) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize detector");
            return CallbackReturn::FAILURE;
        }
        inference_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("~/detections", 10);
        offset_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("~/offset_px", 10);
        offset_m_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/offset_m", 10);
        distance_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/distance_m", 10);
        if (publish_timing_) timing_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/timing", 10);
        RCLCPP_INFO(get_logger(), "Configured successfully");
        return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
        return CallbackReturn::FAILURE;
    }
}

YolosDetectorNode::CallbackReturn YolosDetectorNode::on_activate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "Activating...");
    det_pub_->on_activate();
    if (offset_pub_) offset_pub_->on_activate();
    if (offset_m_pub_) offset_m_pub_->on_activate();
    if (distance_pub_) distance_pub_->on_activate();
    if (timing_pub_) timing_pub_->on_activate();

    if (camera_fx_ <= 0.0 || camera_fy_ <= 0.0 || target_plane_distance_m_ <= 0.0) {
        RCLCPP_ERROR(get_logger(), "camera_fx/camera_fy/target_plane_distance_m must be positive");
        return CallbackReturn::FAILURE;
    }

    auto options = rclcpp::SubscriptionOptions();
    options.callback_group = inference_cb_group_;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "~/image_raw", rclcpp::SensorDataQoS(), std::bind(&YolosDetectorNode::imageCallback, this, std::placeholders::_1),
        options);

    RCLCPP_INFO(get_logger(), "Activated - waiting for images on ~/image_raw");
    return CallbackReturn::SUCCESS;
}

YolosDetectorNode::CallbackReturn YolosDetectorNode::on_deactivate(const rclcpp_lifecycle::State&) {
    image_sub_.reset();
    det_pub_->on_deactivate();
    if (offset_pub_) offset_pub_->on_deactivate();
    if (offset_m_pub_) offset_m_pub_->on_deactivate();
    if (distance_pub_) distance_pub_->on_deactivate();
    if (timing_pub_) timing_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

YolosDetectorNode::CallbackReturn YolosDetectorNode::on_cleanup(const rclcpp_lifecycle::State&) {
    if (detector_) {
        detector_->shutdown();
        detector_.reset();
    }
    det_pub_.reset();
    offset_pub_.reset();
    offset_m_pub_.reset();
    distance_pub_.reset();
    timing_pub_.reset();
    return CallbackReturn::SUCCESS;
}

YolosDetectorNode::CallbackReturn YolosDetectorNode::on_shutdown(const rclcpp_lifecycle::State& s) {
    return on_cleanup(s);
}

void YolosDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!detector_ || !detector_->isInitialized()) {
        return;
    }

    try {
        auto cv_image = cv_bridge::toCvShare(msg, "bgr8");
        processFrame(cv_image->image, msg->header);
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Image conversion failed: %s", e.what());
    }
}

void YolosDetectorNode::processFrame(const cv::Mat& frame, const std_msgs::msg::Header& header) {
    if (!detector_ || !detector_->isInitialized()) return;
    auto t_start = std::chrono::high_resolution_clock::now();
    try {
        auto t_preprocess = std::chrono::high_resolution_clock::now();
        auto detections = detector_->detect(frame, conf_threshold_, nms_threshold_);
        auto t_inference = std::chrono::high_resolution_clock::now();
        det_pub_->publish(conversion::toDetection2DArray(detections, header, frame.cols, frame.rows));
        auto t_postprocess = std::chrono::high_resolution_clock::now();

        const int image_center_x = frame.cols / 2;
        const int image_center_y = frame.rows / 2;

        for (const auto& det : detections) {
            const int center_x = det.bbox.x + det.bbox.width / 2;
            const int center_y = det.bbox.y + det.bbox.height / 2;
            const int offset_x = center_x - image_center_x;  // 左负右正
            const int offset_y = center_y - image_center_y;  // 上负下正

            RCLCPP_INFO(get_logger(), "det=[%s] center=(%d,%d) offset_px=(x:%d,y:%d)", det.class_name.c_str(), center_x,
                        center_y, offset_x, offset_y);

            if (offset_pub_ && offset_pub_->is_activated()) {
                std_msgs::msg::Int32MultiArray offset_msg;
                offset_msg.data = {offset_x, offset_y};
                offset_pub_->publish(offset_msg);
            }

            if (offset_m_pub_ && offset_m_pub_->is_activated()) {
                const double offset_x_m = target_plane_distance_m_ * (static_cast<double>(center_x) - camera_cx_) / camera_fx_;
                const double offset_y_m = target_plane_distance_m_ * (static_cast<double>(center_y) - camera_cy_) / camera_fy_;

                std_msgs::msg::Float64MultiArray offset_m_msg;
                offset_m_msg.data = {offset_x_m, offset_y_m};
                offset_m_pub_->publish(offset_m_msg);

                if (distance_pub_ && distance_pub_->is_activated()) {
                    const double planar_distance_m = std::hypot(offset_x_m, offset_y_m);
                    const double line_distance_m = std::hypot(planar_distance_m, target_plane_distance_m_);

                    std_msgs::msg::Float64MultiArray distance_msg;
                    distance_msg.data = {planar_distance_m, line_distance_m};
                    distance_pub_->publish(distance_msg);
                }
            }
        }
        if (publish_timing_ && timing_pub_ && timing_pub_->is_activated()) {
            std_msgs::msg::Float64MultiArray timing;
            timing.data = {std::chrono::duration<double, std::milli>(t_preprocess - t_start).count(),
                           std::chrono::duration<double, std::milli>(t_inference - t_preprocess).count(),
                           std::chrono::duration<double, std::milli>(t_postprocess - t_inference).count()};
            timing_pub_->publish(timing);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Detection failed: %s", e.what());
    }
}

}  // namespace ros2_yolos_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_yolos_cpp::YolosDetectorNode)

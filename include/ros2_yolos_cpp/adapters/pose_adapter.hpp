// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__ADAPTERS__POSE_ADAPTER_HPP_
#define ROS2_YOLOS_CPP__ADAPTERS__POSE_ADAPTER_HPP_

#include <memory>
#include <vector>
#include "ros2_yolos_cpp/adapters/yolos_adapter_base.hpp"

namespace ros2_yolos_cpp {

class ROS2_YOLOS_CPP_PUBLIC PoseAdapter : public IPoseAdapter {
public:
  PoseAdapter();
  ~PoseAdapter() override;
  PoseAdapter(PoseAdapter&&) noexcept;
  PoseAdapter& operator=(PoseAdapter&&) noexcept;

  bool initialize(const YolosConfig& config) override;
  [[nodiscard]] bool isInitialized() const noexcept override;
  void shutdown() override;
  [[nodiscard]] const std::vector<std::string>& getClassNames() const override;

  std::vector<PoseResult> detect(
    const cv::Mat& image,
    float conf_threshold,
    float nms_threshold) override;

  void drawPoses(
    cv::Mat& image,
    const std::vector<PoseResult>& poses,
    int kpt_radius,
    float kpt_threshold) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros2_yolos_cpp

#endif  // ROS2_YOLOS_CPP__ADAPTERS__POSE_ADAPTER_HPP_

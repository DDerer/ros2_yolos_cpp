// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__ADAPTERS__OBB_ADAPTER_HPP_
#define ROS2_YOLOS_CPP__ADAPTERS__OBB_ADAPTER_HPP_

#include <memory>
#include <vector>
#include "ros2_yolos_cpp/adapters/yolos_adapter_base.hpp"

namespace ros2_yolos_cpp {

class ROS2_YOLOS_CPP_PUBLIC OBBAdapter : public IOBBAdapter {
public:
  OBBAdapter();
  ~OBBAdapter() override;
  OBBAdapter(OBBAdapter&&) noexcept;
  OBBAdapter& operator=(OBBAdapter&&) noexcept;

  bool initialize(const YolosConfig& config) override;
  [[nodiscard]] bool isInitialized() const noexcept override;
  void shutdown() override;
  [[nodiscard]] const std::vector<std::string>& getClassNames() const override;

  std::vector<OBBResult> detect(
    const cv::Mat& image,
    float conf_threshold,
    float nms_threshold,
    int max_det) override;

  void drawDetections(
    cv::Mat& image,
    const std::vector<OBBResult>& detections) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros2_yolos_cpp

#endif  // ROS2_YOLOS_CPP__ADAPTERS__OBB_ADAPTER_HPP_

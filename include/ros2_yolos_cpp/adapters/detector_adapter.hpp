// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__ADAPTERS__DETECTOR_ADAPTER_HPP_
#define ROS2_YOLOS_CPP__ADAPTERS__DETECTOR_ADAPTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ros2_yolos_cpp/adapters/yolos_adapter_base.hpp"

// Forward declaration - actual YOLOs-CPP include only in .cpp
namespace yolos { namespace det { class YOLODetector; } }

namespace ros2_yolos_cpp {

/// @brief Concrete detection adapter wrapping yolos::det::YOLODetector
class ROS2_YOLOS_CPP_PUBLIC DetectorAdapter : public IDetectorAdapter {
public:
  DetectorAdapter();
  ~DetectorAdapter() override;

  // Non-copyable, movable
  DetectorAdapter(const DetectorAdapter&) = delete;
  DetectorAdapter& operator=(const DetectorAdapter&) = delete;
  DetectorAdapter(DetectorAdapter&&) noexcept;
  DetectorAdapter& operator=(DetectorAdapter&&) noexcept;

  bool initialize(const YolosConfig& config) override;
  [[nodiscard]] bool isInitialized() const noexcept override;
  void shutdown() override;
  [[nodiscard]] const std::vector<std::string>& getClassNames() const override;

  std::vector<DetectionResult> detect(
    const cv::Mat& image,
    float conf_threshold,
    float nms_threshold) override;

  void drawDetections(
    cv::Mat& image,
    const std::vector<DetectionResult>& detections) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros2_yolos_cpp

#endif  // ROS2_YOLOS_CPP__ADAPTERS__DETECTOR_ADAPTER_HPP_

// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__ADAPTERS__SEGMENTOR_ADAPTER_HPP_
#define ROS2_YOLOS_CPP__ADAPTERS__SEGMENTOR_ADAPTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ros2_yolos_cpp/adapters/yolos_adapter_base.hpp"

namespace ros2_yolos_cpp {

/// @brief Concrete segmentation adapter wrapping yolos::seg::YOLOSegDetector
class ROS2_YOLOS_CPP_PUBLIC SegmentorAdapter : public ISegmentorAdapter {
public:
  SegmentorAdapter();
  ~SegmentorAdapter() override;

  SegmentorAdapter(const SegmentorAdapter&) = delete;
  SegmentorAdapter& operator=(const SegmentorAdapter&) = delete;
  SegmentorAdapter(SegmentorAdapter&&) noexcept;
  SegmentorAdapter& operator=(SegmentorAdapter&&) noexcept;

  bool initialize(const YolosConfig& config) override;
  [[nodiscard]] bool isInitialized() const noexcept override;
  void shutdown() override;
  [[nodiscard]] const std::vector<std::string>& getClassNames() const override;

  std::vector<SegmentationResult> segment(
    const cv::Mat& image,
    float conf_threshold,
    float nms_threshold) override;

  void drawSegmentations(
    cv::Mat& image,
    const std::vector<SegmentationResult>& segmentations,
    float mask_alpha = 0.5f) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros2_yolos_cpp

#endif  // ROS2_YOLOS_CPP__ADAPTERS__SEGMENTOR_ADAPTER_HPP_

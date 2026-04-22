// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__ADAPTERS__CLASSIFIER_ADAPTER_HPP_
#define ROS2_YOLOS_CPP__ADAPTERS__CLASSIFIER_ADAPTER_HPP_

#include <memory>
#include <vector>
#include "ros2_yolos_cpp/adapters/yolos_adapter_base.hpp"

namespace ros2_yolos_cpp {

class ROS2_YOLOS_CPP_PUBLIC ClassifierAdapter : public IClassifierAdapter {
public:
  ClassifierAdapter();
  ~ClassifierAdapter() override;
  ClassifierAdapter(ClassifierAdapter&&) noexcept;
  ClassifierAdapter& operator=(ClassifierAdapter&&) noexcept;

  bool initialize(const YolosConfig& config) override;
  [[nodiscard]] bool isInitialized() const noexcept override;
  void shutdown() override;
  [[nodiscard]] const std::vector<std::string>& getClassNames() const override;

  ClassificationResult classify(const cv::Mat& image) override;

  void drawResult(
    cv::Mat& image,
    const ClassificationResult& result) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros2_yolos_cpp

#endif  // ROS2_YOLOS_CPP__ADAPTERS__CLASSIFIER_ADAPTER_HPP_

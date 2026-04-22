// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0
// Integration test for SegmentorAdapter and SegmentorNode

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "ros2_yolos_cpp/adapters/segmentor_adapter.hpp"
#include "ros2_yolos_cpp/nodes/segmentor_node.hpp"

namespace ros2_yolos_cpp::test {

class SegmentorTest : public ::testing::Test {
protected:
  static std::string model_path_;
  static std::string labels_path_;
  static std::string test_image_path_;
  
  static void SetUpTestSuite() {
    model_path_ = getEnvOrDefault("YOLOS_SEG_MODEL", "");
    labels_path_ = getEnvOrDefault("YOLOS_SEG_LABELS", "");
    test_image_path_ = getEnvOrDefault("YOLOS_TEST_IMAGE", "");
  }
  
  static std::string getEnvOrDefault(const char* name, const std::string& def) {
    const char* val = std::getenv(name);
    return val ? val : def;
  }
  
  bool hasTestResources() const {
    return !model_path_.empty() && std::filesystem::exists(model_path_);
  }
};

std::string SegmentorTest::model_path_;
std::string SegmentorTest::labels_path_;
std::string SegmentorTest::test_image_path_;

TEST_F(SegmentorTest, AdapterFactoryCreatesInstance) {
  auto adapter = createSegmentorAdapter();
  ASSERT_NE(adapter, nullptr);
}

TEST_F(SegmentorTest, AdapterInitialization) {
  if (!hasTestResources()) GTEST_SKIP();
  
  auto adapter = createSegmentorAdapter();
  YolosConfig config;
  config.model_path = model_path_;
  config.labels_path = labels_path_;
  
  ASSERT_TRUE(adapter->initialize(config));
  EXPECT_TRUE(adapter->isInitialized());
}

TEST_F(SegmentorTest, AdapterSegmentation) {
  if (!hasTestResources()) GTEST_SKIP();
  
  auto adapter = createSegmentorAdapter();
  YolosConfig config;
  config.model_path = model_path_;
  config.labels_path = labels_path_;
  
  ASSERT_TRUE(adapter->initialize(config));
  
  cv::Mat image = cv::imread(test_image_path_);
  if (image.empty()) image = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0,0,0));
  
  auto results = adapter->segment(image, 0.3f, 0.45f);
  // Just check code doesn't crash
  (void)results;
}

TEST_F(SegmentorTest, NodeLifecycleConfiguration) {
  if (!hasTestResources()) GTEST_SKIP();
  
  rclcpp::init(0, nullptr);
  
  rclcpp::NodeOptions options;
  options.append_parameter_override("model_path", model_path_);
  options.append_parameter_override("labels_path", labels_path_);
  
  auto node = std::make_shared<YolosSegmentorNode>(options);
  
  auto state = node->configure();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  
  state = node->activate();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  
  node->deactivate();
  node->cleanup();
  
  rclcpp::shutdown();
}

} // namespace ros2_yolos_cpp::test

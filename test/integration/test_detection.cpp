// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0
// Integration test for DetectorAdapter and DetectorNode

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "ros2_yolos_cpp/adapters/detector_adapter.hpp"
#include "ros2_yolos_cpp/nodes/detector_node.hpp"

namespace ros2_yolos_cpp::test {

class DetectorTest : public ::testing::Test {
protected:
  static std::string model_path_;
  static std::string labels_path_;
  static std::string test_image_path_;
  
  static void SetUpTestSuite() {
    model_path_ = getEnvOrDefault("YOLOS_TEST_MODEL", "");
    labels_path_ = getEnvOrDefault("YOLOS_TEST_LABELS", "");
    test_image_path_ = getEnvOrDefault("YOLOS_TEST_IMAGE", "");
  }
  
  static std::string getEnvOrDefault(const char* name, const std::string& def) {
    const char* val = std::getenv(name);
    return val ? val : def;
  }
  
  bool hasTestResources() const {
    return !model_path_.empty() && 
           !labels_path_.empty() && 
           std::filesystem::exists(model_path_) &&
           std::filesystem::exists(labels_path_);
  }
};

std::string DetectorTest::model_path_;
std::string DetectorTest::labels_path_;
std::string DetectorTest::test_image_path_;

// --- Adapter Tests ---

TEST_F(DetectorTest, AdapterFactoryCreatesInstance) {
  auto adapter = createDetectorAdapter();
  ASSERT_NE(adapter, nullptr);
}

TEST_F(DetectorTest, AdapterInitialization) {
  if (!hasTestResources()) GTEST_SKIP();
  
  auto adapter = createDetectorAdapter();
  YolosConfig config;
  config.model_path = model_path_;
  config.labels_path = labels_path_;
  config.use_gpu = false;
  
  ASSERT_TRUE(adapter->initialize(config));
  EXPECT_TRUE(adapter->isInitialized());
}

TEST_F(DetectorTest, AdapterDetection) {
  if (!hasTestResources() || test_image_path_.empty()) GTEST_SKIP();
  
  auto adapter = createDetectorAdapter();
  YolosConfig config;
  config.model_path = model_path_;
  config.labels_path = labels_path_;
  
  ASSERT_TRUE(adapter->initialize(config));
  
  cv::Mat image = cv::imread(test_image_path_);
  ASSERT_FALSE(image.empty());
  
  auto detections = adapter->detect(image, 0.25f, 0.45f);
  EXPECT_FALSE(detections.empty());
}

// --- Node Tests ---

TEST_F(DetectorTest, NodeLifecycleConfiguration) {
  if (!hasTestResources()) GTEST_SKIP();
  
  // Initialize ROS for this test
  rclcpp::init(0, nullptr);
  
  rclcpp::NodeOptions options;
  options.append_parameter_override("model_path", model_path_);
  options.append_parameter_override("labels_path", labels_path_);
  options.append_parameter_override("use_gpu", false);
  
  auto node = std::make_shared<YolosDetectorNode>(options);
  
  // Test Configure
  auto state = node->configure();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  
  // Test Activate
  state = node->activate();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  
  // Test Cleanup
  state = node->cleanup();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  rclcpp::shutdown();
}

} // namespace ros2_yolos_cpp::test

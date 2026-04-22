// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0
// Integration test for ClassifierAdapter and ClassifierNode

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "ros2_yolos_cpp/adapters/classifier_adapter.hpp"
#include "ros2_yolos_cpp/nodes/classifier_node.hpp"

namespace ros2_yolos_cpp::test {

class ClassifierTest : public ::testing::Test {
protected:
  static std::string model_path_;
  static std::string labels_path_;
  static std::string test_image_path_;
  
  static void SetUpTestSuite() {
    model_path_ = getEnvOrDefault("YOLOS_CLS_MODEL", "");
    labels_path_ = getEnvOrDefault("YOLOS_CLS_LABELS", "");
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

std::string ClassifierTest::model_path_;
std::string ClassifierTest::labels_path_;
std::string ClassifierTest::test_image_path_;

TEST_F(ClassifierTest, AdapterFactoryCreatesInstance) {
  ASSERT_NE(createClassifierAdapter(), nullptr);
}

TEST_F(ClassifierTest, AdapterInitialization) {
  if (!hasTestResources()) GTEST_SKIP();
  auto adapter = createClassifierAdapter();
  YolosConfig config;
  config.model_path = model_path_;
  config.labels_path = labels_path_;
  ASSERT_TRUE(adapter->initialize(config));
}

TEST_F(ClassifierTest, NodeLifecycleConfiguration) {
  if (!hasTestResources()) GTEST_SKIP();
  
  rclcpp::init(0, nullptr);
  
  rclcpp::NodeOptions options;
  options.append_parameter_override("model_path", model_path_);
  options.append_parameter_override("labels_path", labels_path_);
  
  auto node = std::make_shared<YolosClassifierNode>(options);
  
  auto state = node->configure();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  
  state = node->activate();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  
  node->deactivate();
  node->cleanup();
  
  rclcpp::shutdown();
}

} // namespace ros2_yolos_cpp::test

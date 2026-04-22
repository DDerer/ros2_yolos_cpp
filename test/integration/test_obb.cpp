// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0
// Integration test for OBBAdapter and OBBNode

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "ros2_yolos_cpp/adapters/obb_adapter.hpp"
#include "ros2_yolos_cpp/nodes/obb_node.hpp"

namespace ros2_yolos_cpp::test {

class OBBTest : public ::testing::Test {
protected:
  static std::string model_path_;
  static std::string labels_path_;
  static std::string test_image_path_;
  
  static void SetUpTestSuite() {
    model_path_ = getEnvOrDefault("YOLOS_OBB_MODEL", "");
    labels_path_ = getEnvOrDefault("YOLOS_OBB_LABELS", "");
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

std::string OBBTest::model_path_;
std::string OBBTest::labels_path_;
std::string OBBTest::test_image_path_;

TEST_F(OBBTest, AdapterFactoryCreatesInstance) {
  ASSERT_NE(createOBBAdapter(), nullptr);
}

TEST_F(OBBTest, AdapterInitialization) {
  if (!hasTestResources()) GTEST_SKIP();
  auto adapter = createOBBAdapter();
  YolosConfig config;
  config.model_path = model_path_;
  config.labels_path = labels_path_;
  ASSERT_TRUE(adapter->initialize(config));
}

TEST_F(OBBTest, NodeLifecycleConfiguration) {
  if (!hasTestResources()) GTEST_SKIP();
  
  rclcpp::init(0, nullptr);
  
  rclcpp::NodeOptions options;
  options.append_parameter_override("model_path", model_path_);
  options.append_parameter_override("labels_path", labels_path_);
  
  auto node = std::make_shared<YolosOBBNode>(options);
  
  auto state = node->configure();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  
  state = node->activate();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  
  node->deactivate();
  node->cleanup();
  
  rclcpp::shutdown();
}

} // namespace ros2_yolos_cpp::test

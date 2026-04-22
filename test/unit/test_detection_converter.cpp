// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#include <gtest/gtest.h>

#include "ros2_yolos_cpp/conversion/detection_converter.hpp"

namespace ros2_yolos_cpp::conversion {

class DetectionConverterTest : public ::testing::Test {
protected:
  std_msgs::msg::Header header_;
  
  void SetUp() override {
    header_.stamp.sec = 1234567890;
    header_.stamp.nanosec = 123456789;
    header_.frame_id = "camera_link";
  }
};

TEST_F(DetectionConverterTest, SingleDetectionConversion) {
  DetectionResult det;
  det.bbox.x = 100;
  det.bbox.y = 200;
  det.bbox.width = 50;
  det.bbox.height = 80;
  det.confidence = 0.85f;
  det.class_id = 42;
  det.class_name = "person";

  auto msg = toDetection2D(det, header_, 640, 480);

  // Check center conversion (x + w/2, y + h/2)
  EXPECT_DOUBLE_EQ(msg.bbox.center.position.x, 125.0);
  EXPECT_DOUBLE_EQ(msg.bbox.center.position.y, 240.0);
  EXPECT_DOUBLE_EQ(msg.bbox.size_x, 50.0);
  EXPECT_DOUBLE_EQ(msg.bbox.size_y, 80.0);

  // Check hypothesis
  ASSERT_EQ(msg.results.size(), 1u);
  EXPECT_EQ(msg.results[0].hypothesis.class_id, "person");
  EXPECT_FLOAT_EQ(msg.results[0].hypothesis.score, 0.85f);

  // Check header propagation
  EXPECT_EQ(msg.header.frame_id, "camera_link");
}

TEST_F(DetectionConverterTest, EmptyDetectionsArray) {
  std::vector<DetectionResult> empty;
  auto msg = toDetection2DArray(empty, header_, 640, 480);

  EXPECT_TRUE(msg.detections.empty());
  EXPECT_EQ(msg.header.frame_id, "camera_link");
}

TEST_F(DetectionConverterTest, MultipleDetections) {
  std::vector<DetectionResult> dets;
  
  DetectionResult det1;
  det1.bbox = {10, 20, 30, 40};
  det1.confidence = 0.9f;
  det1.class_id = 0;
  det1.class_name = "car";
  dets.push_back(det1);

  DetectionResult det2;
  det2.bbox = {100, 200, 50, 60};
  det2.confidence = 0.7f;
  det2.class_id = 1;
  det2.class_name = "truck";
  dets.push_back(det2);

  auto msg = toDetection2DArray(dets, header_, 1920, 1080);

  ASSERT_EQ(msg.detections.size(), 2u);
  EXPECT_EQ(msg.detections[0].results[0].hypothesis.class_id, "car");
  EXPECT_EQ(msg.detections[1].results[0].hypothesis.class_id, "truck");
}

TEST_F(DetectionConverterTest, CoordinateIntegrity) {
  DetectionResult det;
  det.bbox.x = 0;
  det.bbox.y = 0;
  det.bbox.width = 100;
  det.bbox.height = 100;
  det.confidence = 1.0f;
  det.class_id = 0;

  auto msg = toDetection2D(det, header_, 1000, 1000);

  // Center should be at (50, 50)
  EXPECT_DOUBLE_EQ(msg.bbox.center.position.x, 50.0);
  EXPECT_DOUBLE_EQ(msg.bbox.center.position.y, 50.0);
}

}  // namespace ros2_yolos_cpp::conversion

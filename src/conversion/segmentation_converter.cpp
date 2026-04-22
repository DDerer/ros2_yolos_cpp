// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#include "ros2_yolos_cpp/conversion/segmentation_converter.hpp"
#include "ros2_yolos_cpp/conversion/detection_converter.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace ros2_yolos_cpp {
namespace conversion {

vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<SegmentationResult>& segmentations,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height)
{
  // Reuse detection converter by extracting bbox info
  std::vector<DetectionResult> dets;
  dets.reserve(segmentations.size());
  
  for (const auto& seg : segmentations) {
    DetectionResult det;
    det.bbox = seg.bbox;
    det.confidence = seg.confidence;
    det.class_id = seg.class_id;
    det.class_name = seg.class_name;
    dets.push_back(det);
  }

  return toDetection2DArray(dets, header, image_width, image_height);
}

sensor_msgs::msg::Image toMaskImage(
  const cv::Mat& mask,
  const std_msgs::msg::Header& header)
{
  cv_bridge::CvImage cv_img;
  cv_img.header = header;
  cv_img.encoding = sensor_msgs::image_encodings::MONO8;
  
  if (mask.type() == CV_8UC1) {
    cv_img.image = mask;
  } else {
    mask.convertTo(cv_img.image, CV_8UC1);
  }

  return *cv_img.toImageMsg();
}

sensor_msgs::msg::Image toCombinedMaskImage(
  const std::vector<SegmentationResult>& segmentations,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height)
{
  // Create a combined class-labeled mask
  cv::Mat combined = cv::Mat::zeros(image_height, image_width, CV_8UC1);

  for (const auto& seg : segmentations) {
    if (!seg.mask.empty()) {
      cv::Mat seg_mask;
      if (seg.mask.size() != combined.size()) {
        cv::resize(seg.mask, seg_mask, combined.size());
      } else {
        seg_mask = seg.mask;
      }

      // Label each pixel with class_id + 1 (0 = background)
      for (int y = 0; y < combined.rows; ++y) {
        for (int x = 0; x < combined.cols; ++x) {
          if (seg_mask.at<uchar>(y, x) > 0) {
            combined.at<uchar>(y, x) = static_cast<uchar>(seg.class_id + 1);
          }
        }
      }
    }
  }

  return toMaskImage(combined, header);
}

}  // namespace conversion
}  // namespace ros2_yolos_cpp

// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#ifndef ROS2_YOLOS_CPP__VISIBILITY_CONTROL_HPP_
#define ROS2_YOLOS_CPP__VISIBILITY_CONTROL_HPP_

// =============================================================================
// DLL Export/Import Macros for Cross-Platform Component Library
// =============================================================================

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROS2_YOLOS_CPP_EXPORT __attribute__((dllexport))
    #define ROS2_YOLOS_CPP_IMPORT __attribute__((dllimport))
  #else
    #define ROS2_YOLOS_CPP_EXPORT __declspec(dllexport)
    #define ROS2_YOLOS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROS2_YOLOS_CPP_BUILDING_DLL
    #define ROS2_YOLOS_CPP_PUBLIC ROS2_YOLOS_CPP_EXPORT
  #else
    #define ROS2_YOLOS_CPP_PUBLIC ROS2_YOLOS_CPP_IMPORT
  #endif
  #define ROS2_YOLOS_CPP_LOCAL
#else
  #define ROS2_YOLOS_CPP_EXPORT __attribute__((visibility("default")))
  #define ROS2_YOLOS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define ROS2_YOLOS_CPP_PUBLIC __attribute__((visibility("default")))
    #define ROS2_YOLOS_CPP_LOCAL __attribute__((visibility("hidden")))
  #else
    #define ROS2_YOLOS_CPP_PUBLIC
    #define ROS2_YOLOS_CPP_LOCAL
  #endif
#endif

#endif  // ROS2_YOLOS_CPP__VISIBILITY_CONTROL_HPP_

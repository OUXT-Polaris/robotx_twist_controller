#ifndef ROBOTX_TWIST_CONTROLLER__ROBOTX_TWIST_CONTROLLER_COMPONENT_HPP_
#define ROBOTX_TWIST_CONTROLLER__ROBOTX_TWIST_CONTROLLER_COMPONENT_HPP_

// Copyright (c) 2020 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_EXPORT __attribute__((dllexport))
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_EXPORT __declspec(dllexport)
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_BUILDING_DLL
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_PUBLIC \
  ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_EXPORT
#else
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_PUBLIC \
  ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_IMPORT
#endif
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_PUBLIC_TYPE \
  ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_PUBLIC
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_LOCAL
#else
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_PUBLIC
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_LOCAL
#endif
#define ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace robotx_twist_controller
{
class RobotXTwistControllerComponent : public rclcpp::Node
{
public:
  ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_PUBLIC
  explicit RobotXTwistControllerComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_twist_sub_;
};
}

#endif  // ROBOTX_TWIST_CONTROLLER__ROBOTX_TWIST_CONTROLLER_COMPONENT_HPP_

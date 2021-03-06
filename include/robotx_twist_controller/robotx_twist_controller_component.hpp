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

#ifndef ROBOTX_TWIST_CONTROLLER__ROBOTX_TWIST_CONTROLLER_COMPONENT_HPP_
#define ROBOTX_TWIST_CONTROLLER__ROBOTX_TWIST_CONTROLLER_COMPONENT_HPP_

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

#include <robotx_twist_controller/optimization_problem.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/optional.hpp>

#include <string>

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
  void currentTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data);
  void targetTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data);
  geometry_msgs::msg::PoseStamped cog_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  boost::optional<geometry_msgs::msg::Twist> current_twist_, target_twist_;
  void update();
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;
  geometry_msgs::msg::TransformStamped getTransformFromCogFrame(
    std::string frame_id,
    builtin_interfaces::msg::Time stamp);
  std::string left_engine_link_, right_engine_link_;
  double current_left_turust_, current_right_turust_, current_alpha_;
};
}  // namespace robotx_twist_controller

#endif  // ROBOTX_TWIST_CONTROLLER__ROBOTX_TWIST_CONTROLLER_COMPONENT_HPP_

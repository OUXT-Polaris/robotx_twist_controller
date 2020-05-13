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

#include <robotx_twist_controller/robotx_twist_controller_component.hpp>

namespace robotx_twist_controller
{
RobotXTwistControllerComponent::RobotXTwistControllerComponent(const rclcpp::NodeOptions & options)
: Node("robotx_twist_controller", options), broadcaster_(this), buffer_(get_clock()), listener_(
    buffer_)
{
  using namespace std::chrono_literals;
  declare_parameter("cog_parent_frame_id", "base_link");
  get_parameter("cog_parent_frame_id", cog_pose_.header.frame_id);
  declare_parameter("cog_position_x", 0.0);
  get_parameter("cog_position_x", cog_pose_.pose.position.x);
  declare_parameter("cog_position_y", 0.0);
  get_parameter("cog_position_y", cog_pose_.pose.position.y);
  declare_parameter("cog_position_z", 0.0);
  get_parameter("cog_position_z", cog_pose_.pose.position.z);
  declare_parameter("cog_orientation_x", 0.0);
  get_parameter("cog_orientation_x", cog_pose_.pose.orientation.x);
  declare_parameter("cog_orientation_y", 0.0);
  get_parameter("cog_orientation_y", cog_pose_.pose.orientation.y);
  declare_parameter("cog_orientation_z", 0.0);
  get_parameter("cog_orientation_z", cog_pose_.pose.orientation.z);
  declare_parameter("cog_orientation_w", 1.0);
  get_parameter("cog_orientation_w", cog_pose_.pose.orientation.w);
  target_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "target_twist", 1,
    std::bind(&RobotXTwistControllerComponent::targetTwistCallback, this, std::placeholders::_1));
  current_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "current_twist", 1,
    std::bind(&RobotXTwistControllerComponent::currentTwistCallback, this, std::placeholders::_1));
  timer_ =
    this->create_wall_timer(10ms, std::bind(&RobotXTwistControllerComponent::update, this));
}

void RobotXTwistControllerComponent::update()
{
  // broadcast tf
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = get_clock()->now();
  transform_stamped.header.frame_id = cog_pose_.header.frame_id;
  transform_stamped.child_frame_id = "cog";
  transform_stamped.transform.translation.x = cog_pose_.pose.position.x;
  transform_stamped.transform.translation.y = cog_pose_.pose.position.y;
  transform_stamped.transform.translation.z = cog_pose_.pose.position.z;
  transform_stamped.transform.rotation = cog_pose_.pose.orientation;
  broadcaster_.sendTransform(transform_stamped);
}

void RobotXTwistControllerComponent::currentTwistCallback(
  const geometry_msgs::msg::Twist::SharedPtr data)
{
  current_twist_ = *data;
}

void RobotXTwistControllerComponent::targetTwistCallback(
  const geometry_msgs::msg::Twist::SharedPtr data)
{
  target_twist_ = *data;
}
}

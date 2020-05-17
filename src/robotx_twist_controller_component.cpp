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
#include <ceres/ceres.h>
#include <string>

namespace robotx_twist_controller
{
RobotXTwistControllerComponent::RobotXTwistControllerComponent(const rclcpp::NodeOptions & options)
: Node("robotx_twist_controller", options), buffer_(get_clock()), listener_(
    buffer_), broadcaster_(this)
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
  declare_parameter("left_engine_link", "left_engine_link");
  get_parameter("left_engine_link", left_engine_link_);
  declare_parameter("right_engine_link", "right_engine_link");
  get_parameter("right_engine_link", right_engine_link_);
  current_left_turust_ = 0.0;
  current_right_turust_ = 0.0;
  current_alpha_ = 0.0;
  target_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "target_twist", 1,
    std::bind(&RobotXTwistControllerComponent::targetTwistCallback, this, std::placeholders::_1));
  current_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "current_twist", 1,
    std::bind(&RobotXTwistControllerComponent::currentTwistCallback, this, std::placeholders::_1));
  timer_ =
    this->create_wall_timer(20ms, std::bind(&RobotXTwistControllerComponent::update, this));
}

geometry_msgs::msg::TransformStamped RobotXTwistControllerComponent::getTransformFromCogFrame(
  std::string frame_id, builtin_interfaces::msg::Time stamp)
{
  tf2::TimePoint time_point = tf2::TimePoint(
    std::chrono::seconds(stamp.sec) +
    std::chrono::nanoseconds(stamp.nanosec));
  geometry_msgs::msg::TransformStamped transform_stamped =
    buffer_.lookupTransform("cog", frame_id, time_point, tf2::durationFromSec(1.0));
  return transform_stamped;
}

void RobotXTwistControllerComponent::update()
{
  // broadcast tf
  geometry_msgs::msg::TransformStamped transform_stamped;
  auto stamp = get_clock()->now();
  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = cog_pose_.header.frame_id;
  transform_stamped.child_frame_id = "cog";
  transform_stamped.transform.translation.x = cog_pose_.pose.position.x;
  transform_stamped.transform.translation.y = cog_pose_.pose.position.y;
  transform_stamped.transform.translation.z = cog_pose_.pose.position.z;
  transform_stamped.transform.rotation = cog_pose_.pose.orientation;
  broadcaster_.sendTransform(transform_stamped);

  if (!current_twist_ || !target_twist_) {
    return;
  }

  double x_desired = (target_twist_.get().linear.x - current_twist_.get().linear.x) / 50;
  double y_desired = (target_twist_.get().linear.y - current_twist_.get().linear.y) / 50;
  double n_desired = (target_twist_.get().angular.z - current_twist_.get().angular.z) / 50;

  auto left_engine_transform = getTransformFromCogFrame(left_engine_link_, stamp);
  auto right_engine_transform = getTransformFromCogFrame(left_engine_link_, stamp);
  double r1x = -1 * left_engine_transform.transform.translation.x;
  double r2x = -1 * right_engine_transform.transform.translation.x;
  double r1y = left_engine_transform.transform.translation.y;
  double r2y = right_engine_transform.transform.translation.y;

  using ceres::AutoDiffCostFunction;
  using ceres::CostFunction;
  using ceres::Problem;
  using ceres::Solver;
  using ceres::Solve;
  ceres::Problem problem;
  double t1 = current_left_turust_;
  double t2 = current_right_turust_;
  double alpha = current_alpha_;
  problem.AddResidualBlock(FuncX::Create(x_desired), NULL, &t1, &t2, &alpha);
  problem.AddResidualBlock(FuncY::Create(y_desired), NULL, &t1, &t2, &alpha);
  problem.AddResidualBlock(FuncN::Create(n_desired, r1x, r1y, r2x, r2y), NULL, &t1, &t2, &alpha);
  problem.AddResidualBlock(FuncAlpha::Create(), NULL, &alpha);
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  current_left_turust_ = t1;
  current_right_turust_ = t2;
  current_alpha_ = alpha;
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
}  // namespace robotx_twist_controller

// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2022 Joshua Wallace
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

#ifndef NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_HPP_
#define NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behaviors
{

/**
 * @class nav2_behaviors::DriveOnHeading
 * @brief An action server Behavior for spinning in
 */
template<typename ActionT = nav2_msgs::action::DriveOnHeading>  // 设置模板的默认类型
class DriveOnHeading : public TimedBehavior<ActionT>
{
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  /**
   * @brief A constructor for nav2_behaviors::DriveOnHeading
   */
  DriveOnHeading()
  : TimedBehavior<ActionT>(),
    feedback_(std::make_shared<typename ActionT::Feedback>()),
    command_x_(0.0),
    command_speed_(0.0),
    simulate_ahead_time_(0.0)
  {
  }

  ~DriveOnHeading() = default;

  /**
   * @brief Initialization to run behavior   // 接手并判断命令 目标点、速度及时间  获取机器人初始位姿
   * @param command Goal to execute
   * @return Status of behavior
   */
  ResultStatus onRun(const std::shared_ptr<const typename ActionT::Goal> command) override
  {
    if (command->target.y != 0.0 || command->target.z != 0.0) {
      RCLCPP_INFO(
        this->logger_,
        "DrivingOnHeading in Y and Z not supported, will only move in X.");
      return ResultStatus{Status::FAILED, ActionT::Result::INVALID_INPUT};
    }

    // Ensure that both the speed and direction have the same sign
    if (!((command->target.x > 0.0) == (command->speed > 0.0)) ) {
      RCLCPP_ERROR(this->logger_, "Speed and command sign did not match");
      return ResultStatus{Status::FAILED, ActionT::Result::INVALID_INPUT};
    }

    command_x_ = command->target.x;
    command_speed_ = command->speed;
    command_time_allowance_ = command->time_allowance;

    end_time_ = this->clock_->now() + command_time_allowance_;

    if (!nav2_util::getCurrentPose(
        initial_pose_, *this->tf_, this->local_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
      return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR};
    }

    return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE};
  }

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  ResultStatus onCycleUpdate() override
  {
    rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {  // 超时 停车
      this->stopRobot();
      RCLCPP_WARN(
        this->logger_,
        "Exceeded time allowance before reaching the DriveOnHeading goal - Exiting DriveOnHeading");
      return ResultStatus{Status::FAILED, ActionT::Result::NONE};
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
        current_pose, *this->tf_, this->local_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
      return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR};
    }

    double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
    double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
    double distance = hypot(diff_x, diff_y);

    feedback_->distance_traveled = distance;
    this->action_server_->publish_feedback(feedback_);  // 反馈本次任务行驶的距离

    if (distance >= std::fabs(command_x_)) {  // 超距停止
      this->stopRobot();
      return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE};
    }

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.y = 0.0;
    cmd_vel->angular.z = 0.0;
    cmd_vel->linear.x = command_speed_;

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = current_pose.pose.position.x;
    pose2d.y = current_pose.pose.position.y;
    pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

    if (!isCollisionFree(distance, cmd_vel.get(), pose2d)) {  // 碰撞检测
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
      return ResultStatus{Status::FAILED, ActionT::Result::COLLISION_AHEAD};
    }

    this->vel_pub_->publish(std::move(cmd_vel));  // 控制指令下发

    return ResultStatus{Status::RUNNING, ActionT::Result::NONE};
  }

  /**
   * @brief Method to determine the required costmap info
   * @return costmap resources needed
   */
  CostmapInfoType getResourceInfo() override {return CostmapInfoType::LOCAL;}

protected:
  /**
   * @brief Check if pose is collision free
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   * 
   * 
   * 检查给定的机器人位姿在按照特定的速度和距离模拟运动一段时长后，是否会发生碰撞。
   * 它通过逐步模拟机器人的运动过程，在每个模拟的时间步长内检查机器人所在位置是否处于无碰撞状态，
   * 以此来判断整个模拟运动过程是否能保持无碰撞，最终返回一个布尔值来表明是否无碰撞
   */
  bool isCollisionFree(
    const double & distance,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d)
  {
    // Simulate ahead by simulate_ahead_time_ in this->cycle_frequency_ increments
    int cycle_count = 0;
    double sim_position_change;
    const double diff_dist = abs(command_x_) - distance;
    const int max_cycle_count = static_cast<int>(this->cycle_frequency_ * simulate_ahead_time_);
    geometry_msgs::msg::Pose2D init_pose = pose2d;
    bool fetch_data = true;

    while (cycle_count < max_cycle_count) {
      sim_position_change = cmd_vel->linear.x * (cycle_count / this->cycle_frequency_);  // 这里为什么不用   cycle_count/max_cycle_count
      pose2d.x = init_pose.x + sim_position_change * cos(init_pose.theta);
      pose2d.y = init_pose.y + sim_position_change * sin(init_pose.theta);
      cycle_count++;

      if (diff_dist - abs(sim_position_change) <= 0.) {  // 超过 目标距离 终止
        break;
      }

      if (!this->local_collision_checker_->isCollisionFree(pose2d, fetch_data)) {
        return false;
      }
      fetch_data = false;
    }
    return true;
  }

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override
  {
    auto node = this->node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      "simulate_ahead_time", rclcpp::ParameterValue(2.0));
    node->get_parameter("simulate_ahead_time", simulate_ahead_time_);
  }

  typename ActionT::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  double command_x_;
  double command_speed_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
  double simulate_ahead_time_;
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_HPP_

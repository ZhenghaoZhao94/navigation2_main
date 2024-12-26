// Copyright (c) 2018 Intel Corporation
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

#include "nav2_bt_navigator/bt_navigator.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator(rclcpp::NodeOptions options)
: nav2_util::LifecycleNode("bt_navigator", "",
    options.automatically_declare_parameters_from_overrides(true)),
  class_loader_("nav2_core", "nav2_core::NavigatorBase")
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
    "nav2_smooth_path_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_assisted_teleop_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_drive_on_heading_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_globally_updated_goal_condition_bt_node",
    "nav2_is_path_valid_condition_bt_node",
    "nav2_are_error_codes_active_condition_bt_node",
    "nav2_would_a_controller_recovery_help_condition_bt_node",
    "nav2_would_a_planner_recovery_help_condition_bt_node",
    "nav2_would_a_smoother_recovery_help_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_truncate_path_local_action_bt_node",
    "nav2_goal_updater_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_path_expiring_timer_condition",
    "nav2_distance_traveled_condition_bt_node",
    "nav2_single_trigger_bt_node",
    "nav2_goal_updated_controller_bt_node",
    "nav2_is_battery_low_condition_bt_node",
    "nav2_navigate_through_poses_action_bt_node",
    "nav2_navigate_to_pose_action_bt_node",
    "nav2_remove_passed_goals_action_bt_node",
    "nav2_planner_selector_bt_node",
    "nav2_controller_selector_bt_node",
    "nav2_goal_checker_selector_bt_node",
    "nav2_controller_cancel_bt_node",
    "nav2_path_longer_on_approach_bt_node",
    "nav2_wait_cancel_bt_node",
    "nav2_spin_cancel_bt_node",
    "nav2_assisted_teleop_cancel_bt_node",
    "nav2_back_up_cancel_bt_node",
    "nav2_drive_on_heading_cancel_bt_node",
    "nav2_is_battery_charging_condition_bt_node"
  };

  declare_parameter_if_not_declared(
    this, "plugin_lib_names", rclcpp::ParameterValue(plugin_libs));
  declare_parameter_if_not_declared(
    this, "transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    this, "global_frame", rclcpp::ParameterValue(std::string("map")));
  declare_parameter_if_not_declared(
    this, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter_if_not_declared(
    this, "odom_topic", rclcpp::ParameterValue(std::string("odom")));
}

BtNavigator::~BtNavigator()
{
}

nav2_util::CallbackReturn
BtNavigator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());  // 为 tf2_ros::Buffer 设置定时器接口，使得 tf2_ros::Buffer 能够利用 ROS2 的定时器机制来处理一些与时间相关的任务，比如定期清理过期的坐标变换缓存等。
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);  // 独立线程
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_base_frame").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  odom_topic_ = get_parameter("odom_topic").as_string();

  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  nav2_core::FeedbackUtils feedback_utils;
  feedback_utils.tf = tf_;
  feedback_utils.global_frame = global_frame_;
  feedback_utils.robot_frame = robot_frame_;
  feedback_utils.transform_tolerance = transform_tolerance_;

  // Odometry smoother object for getting current speed
  auto node = shared_from_this();  // 通过调用 shared_from_this 获取当前节点的共享指针 node
  odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(node, 0.3, odom_topic_);

  // Navigator defaults
  const std::vector<std::string> default_navigator_ids = {
    "navigate_to_pose",
    "navigate_through_poses"
  };
  const std::vector<std::string> default_navigator_types = {
    "nav2_bt_navigator/NavigateToPoseNavigator",
    "nav2_bt_navigator/NavigateThroughPosesNavigator"
  };

  std::vector<std::string> navigator_ids;
  declare_parameter_if_not_declared(
    node, "navigators",
    rclcpp::ParameterValue(default_navigator_ids));

  get_parameter("navigators", navigator_ids);

  if (navigator_ids == default_navigator_ids) {
    for (size_t i = 0; i < default_navigator_ids.size(); ++i) {
      declare_parameter_if_not_declared(
        node, default_navigator_ids[i] + ".plugin",
        rclcpp::ParameterValue(default_navigator_types[i]));
    }
  }

  // Load navigator plugins
  for (size_t i = 0; i != navigator_ids.size(); i++) {
    try {
      std::string navigator_type = nav2_util::get_plugin_type_param(node, navigator_ids[i]);
      RCLCPP_INFO(
        get_logger(), "Creating navigator id %s of type %s",
        navigator_ids[i].c_str(), navigator_type.c_str());
        
      navigators_.push_back(class_loader_.createUniqueInstance(navigator_type));   // 加载并配置插件
      if (!navigators_.back()->on_configure(
          node, plugin_lib_names, feedback_utils,
          &plugin_muxer_, odom_smoother_))
      {
        return nav2_util::CallbackReturn::FAILURE;
      }
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create navigator id %s."
        " Exception: %s", navigator_ids[i].c_str(), ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  for (size_t i = 0; i != navigators_.size(); i++) {     // 激活所有插件
    if (!navigators_[i]->on_activate()) {
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)   // 关闭所有插件
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  for (size_t i = 0; i != navigators_.size(); i++) {
    if (!navigators_[i]->on_deactivate()) {
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)   // 清理所有插件  清空插件容器
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

  for (size_t i = 0; i != navigators_.size(); i++) {
    if (!navigators_[i]->on_cleanup()) {
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  navigators_.clear();
  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace nav2_bt_navigator

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_bt_navigator::BtNavigator)

/*
RCLCPP_COMPONENTS_REGISTER_NODE 是一个宏，这里调用它的目的是
将 nav2_bt_navigator::BtNavigator 这个类所代表的节点注册为一个 ROS2 组件。
在 ROS2 的组件化架构中，通过这样的注册操作，使得这个节点（具体来说是其对应的类实现）
能够被 class_loader 机制所识别和加载。class_loader 可以在运行时动态地加载库并实例化相应的节点组件，
就如同创建了一个入口点，当整个 ROS2 系统所在的进程加载包含该节点组件代码的库时，借助这个注册信息，
系统可以准确地找到并启用 nav2_bt_navigator::BtNavigator 这个节点，
使其能够参与到 ROS2 的消息传递、服务调用等各种运行时的交互操作中，
进而融入整个 ROS2 应用生态中发挥其特定的导航相关功能（因为从命名来看可能和导航行为树相关）。

 */
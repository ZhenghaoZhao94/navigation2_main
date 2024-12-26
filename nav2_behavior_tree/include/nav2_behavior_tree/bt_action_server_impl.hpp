// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_

#include <memory>
#include <string>
#include <fstream>
#include <set>
#include <exception>
#include <vector>
#include <limits>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behavior_tree
{

template<class ActionT>
BtActionServer<ActionT>::BtActionServer(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & action_name,                           //  ex navigate_to_pose
  const std::vector<std::string> & plugin_lib_names,         // 所有 行为树 plugin name
  const std::string & default_bt_xml_filename,               // 行为树 xml 文件路径
  OnGoalReceivedCallback on_goal_received_callback,
  OnLoopCallback on_loop_callback,
  OnPreemptCallback on_preempt_callback,
  OnCompletionCallback on_completion_callback)
: action_name_(action_name),
  default_bt_xml_filename_(default_bt_xml_filename),
  plugin_lib_names_(plugin_lib_names),
  node_(parent),
  on_goal_received_callback_(on_goal_received_callback),
  on_loop_callback_(on_loop_callback),
  on_preempt_callback_(on_preempt_callback),
  on_completion_callback_(on_completion_callback)
{
  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare this node's parameters
  if (!node->has_parameter("bt_loop_duration")) {
    node->declare_parameter("bt_loop_duration", 10);
  }
  if (!node->has_parameter("default_server_timeout")) {
    node->declare_parameter("default_server_timeout", 20);
  }
  if (!node->has_parameter("action_server_result_timeout")) {
    node->declare_parameter("action_server_result_timeout", 900.0);
  }

  std::vector<std::string> error_code_names = {
    "follow_path_error_code",
    "compute_path_error_code"
  };
  // 确保节点参数 error_code_names 被正确声明和初始化。如果该参数未被声明或未设置值，则通过默认值或用户提供的值对其进行初始化
  if (!node->has_parameter("error_code_names")) {
    const rclcpp::ParameterValue value = node->declare_parameter(
      "error_code_names",
      rclcpp::PARAMETER_STRING_ARRAY);
    if (value.get_type() == rclcpp::PARAMETER_NOT_SET) {   // 当一个参数被声明但未提供默认值时，其类型为 rclcpp::PARAMETER_NOT_SET。
      std::string error_codes_str;
      for (const auto & error_code : error_code_names) {
        error_codes_str += " " + error_code;  // log 输出
      }
      RCLCPP_WARN_STREAM(
        logger_, "Error_code parameters were not set. Using default values of:"
          << error_codes_str + "\n"
          << "Make sure these match your BT and there are not other sources of error codes you"
          "reported to your application");
      rclcpp::Parameter error_code_names_param("error_code_names", error_code_names);  // 创建一个 ROS 2 参数对象，并初始化它的名称和值
      node->set_parameter(error_code_names_param);  // 将参数对象 error_code_names_param 的值设置到节点的参数服务器中。
    } else {
      error_code_names = value.get<std::vector<std::string>>();
      std::string error_codes_str;
      for (const auto & error_code : error_code_names) {
        error_codes_str += " " + error_code;
      }
      RCLCPP_INFO_STREAM(logger_, "Error_code parameters were set to:" << error_codes_str);
    }
  }
}

template<class ActionT>
BtActionServer<ActionT>::~BtActionServer()
{}

template<class ActionT>
bool BtActionServer<ActionT>::on_configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Name client node after action name
  std::string client_node_name = action_name_;
  std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');  // 将 client_node_name 中 所有出现的字符 '/' 替换成字符 '_'。
  // Use suffix '_rclcpp_node' to keep parameter file consistency #1773
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r",
      std::string("__node:=") +
      std::string(node->get_name()) + "_" + client_node_name + "_rclcpp_node",
      "-p",
      "use_sim_time:=" +
      std::string(node->get_parameter("use_sim_time").as_bool() ? "true" : "false"),
      "--"});

  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  // Declare parameters for common client node applications to share with BT nodes
  // Declare if not declared in case being used an external application, then copying
  // all of the main node's parameters to the client for BT nodes to obtain
  nav2_util::declare_parameter_if_not_declared(
    node, "global_frame", rclcpp::ParameterValue(std::string("map")));
  nav2_util::declare_parameter_if_not_declared(
    node, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  nav2_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  rclcpp::copy_all_parameter_values(node, client_node_);  // 将传入主节点的所有参数 复制到 client_node_中

  // set the timeout in seconds for the action server to discard goal handles if not finished
  double action_server_result_timeout;
  node->get_parameter("action_server_result_timeout", action_server_result_timeout);

  /*
    rcl_action_server_options_t 类型：
    这是 rcl（ROS2 的客户端库核心部分）中定义的一个结构体类型，用于存储动作服务器的各种选项配置。
    这些选项可以控制动作服务器的行为，例如超时时间、回调函数的执行方式等。
    通过设置这个结构体中的不同成员，可以定制动作服务器以满足特定的应用需求。

    rcl_action_server_get_default_options 函数：
    该函数用于获取动作服务器选项的默认值。它返回一个 rcl_action_server_options_t 类型的结构体，其中包含了动作服务器的默认配置选项。
    这些默认选项通常是根据 ROS2 的设计原则和常见使用场景设置的，但在很多情况下，
    开发者需要根据具体的应用需求对这些选项进行修改，就像在这段代码中后续对超时时间进行重新设置一样。-+
    RCL_S_TO_NS 宏：
    RCL_S_TO_NS 是一个用于将秒转换为纳秒的宏（从命名推测其功能，实际在 rcl 库中应该有相应的定义实现）。
    在 ROS2 的时间相关处理中，时间通常以纳秒为单位进行精确表示，因为很多系统操作和定时要求需要高精度的时间控制。
    在这里，将从参数服务器获取到的以秒为单位的 
    action_server_result_timeout 值通过 RCL_S_TO_NS 宏转换为纳秒后，
    赋值给 server_options.result_timeout.nanoseconds，从而正确设置动作服务器的结果超时时间选项。
  */
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  action_server_ = std::make_shared<ActionServer>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    action_name_, std::bind(&BtActionServer<ActionT>::executeCallback, this),
    nullptr, std::chrono::milliseconds(500), false, server_options);

  // Get parameters for BT timeouts
  int bt_loop_duration;
  node->get_parameter("bt_loop_duration", bt_loop_duration);
  bt_loop_duration_ = std::chrono::milliseconds(bt_loop_duration);
  int default_server_timeout;
  node->get_parameter("default_server_timeout", default_server_timeout);
  default_server_timeout_ = std::chrono::milliseconds(default_server_timeout);
  int wait_for_service_timeout;
  node->get_parameter("wait_for_service_timeout", wait_for_service_timeout);
  wait_for_service_timeout_ = std::chrono::milliseconds(wait_for_service_timeout);

  // Get error code id names to grab off of the blackboard
  error_code_names_ = node->get_parameter("error_code_names").as_string_array();

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);  // NOLINT 
  blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout_);

  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_activate()
{
  if (!loadBehaviorTree(default_bt_xml_filename_)) {
    RCLCPP_ERROR(logger_, "Error loading XML file: %s", default_bt_xml_filename_.c_str());
    return false;
  }
  action_server_->activate();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_deactivate()
{
  action_server_->deactivate();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_cleanup()
{
  client_node_.reset();
  action_server_.reset();
  topic_logger_.reset();
  plugin_lib_names_.clear();
  current_bt_xml_filename_.clear();
  blackboard_.reset();
  bt_->haltAllActions(tree_);
  bt_.reset();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // Empty filename is default for backward compatibility
  auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

  // Use previous BT if it is the existing one
  if (current_bt_xml_filename_ == filename) {
    RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
    return true;
  }

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", filename.c_str());
    return false;
  }

  // Create the Behavior Tree from the XML input
  try {
    tree_ = bt_->createTreeFromFile(filename, blackboard_);
    for (auto & blackboard : tree_.blackboard_stack) {   //  ???
      blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_);
      blackboard->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
      blackboard->set<std::chrono::milliseconds>(
        "wait_for_service_timeout",
        wait_for_service_timeout_);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception when loading BT: %s", e.what());
    return false;
  }

  topic_logger_ = std::make_unique<RosTopicLogger>(client_node_, tree_);

  current_bt_xml_filename_ = filename;
  return true;
}

template<class ActionT>
void BtActionServer<ActionT>::executeCallback()  //  action接受目标后 执行主程序
{
  if (!on_goal_received_callback_(action_server_->get_current_goal())) {   // 初始化目标点 加载行为树(action goal 指定行为树路径名字)
    action_server_->terminate_current();
    cleanErrorCodes();
    return;
  }

  auto is_canceling = [&]() {
      if (action_server_ == nullptr) {
        RCLCPP_DEBUG(logger_, "Action server unavailable. Canceling.");
        return true;
      }
      if (!action_server_->is_server_active()) {
        RCLCPP_DEBUG(logger_, "Action server is inactive. Canceling.");
        return true;
      }
      return action_server_->is_cancel_requested();
    };

  auto on_loop = [&]() {              // on_preempt_callback_ 是一个 函数对象，在定义时只是一个占位符，可能未绑定任何具体函数。
      if (action_server_->is_preempt_requested() && on_preempt_callback_) {  // 在绑定之前，on_preempt_callback_ 为空，因此需要在调用前检查 if (on_preempt_callback_)，以防止调用空的回调函数。
        on_preempt_callback_(action_server_->get_pending_goal());
      }
      topic_logger_->flush();  // log 处理，未看
      on_loop_callback_();
    };

  // Execute the BT that was previously created in the configure step
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);  // ???

  // Make sure that the Bt is not in a running state from a previous execution
  // note: if all the ControlNodes are implemented correctly, this is not needed.
  bt_->haltAllActions(tree_);  // 结束所有行为树任务

  // Give server an opportunity to populate the result message or simple give
  // an indication that the action is complete.
  auto result = std::make_shared<typename ActionT::Result>();

  populateErrorCode(result);

  on_completion_callback_(result, rc);

  switch (rc) {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(logger_, "Goal succeeded");
      action_server_->succeeded_current(result);
      break;

    case nav2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(logger_, "Goal failed");
      action_server_->terminate_current(result);
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(logger_, "Goal canceled");
      action_server_->terminate_all(result);
      break;
  }

  cleanErrorCodes();
}

template<class ActionT>
void BtActionServer<ActionT>::populateErrorCode(
  typename std::shared_ptr<typename ActionT::Result> result)
{
  int highest_priority_error_code = std::numeric_limits<int>::max();
  for (const auto & error_code : error_code_names_) {
    try {
      int current_error_code = blackboard_->get<int>(error_code);
      if (current_error_code != 0 && current_error_code < highest_priority_error_code) {
        highest_priority_error_code = current_error_code;
      }
    } catch (...) {
      RCLCPP_DEBUG(
        logger_,
        "Failed to get error code: %s from blackboard",
        error_code.c_str());
    }
  }

  if (highest_priority_error_code != std::numeric_limits<int>::max()) {
    result->error_code = highest_priority_error_code;
  }
}

template<class ActionT>
void BtActionServer<ActionT>::cleanErrorCodes()
{
  for (const auto & error_code : error_code_names_) {
    blackboard_->set<unsigned short>(error_code, 0);  //NOLINT
  }
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_

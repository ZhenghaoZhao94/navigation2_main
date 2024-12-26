// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

ParametersHandler::ParametersHandler(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent)
{
  node_ = parent;
  auto node = node_.lock();
  node_name_ = node->get_name();
  logger_ = node->get_logger();
}

void ParametersHandler::start()
{
  auto node = node_.lock();
  on_set_param_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParametersHandler::dynamicParamsCallback, this,
      std::placeholders::_1));

  auto get_param = getParamGetter(node_name_);
  get_param(verbose_, "verbose", false);
}
//在动态参数发生变化时，根据参数的名称查找相应的回调函数进行处理，并记录操作的结果。函数中通过执行预回调和后续回调函数，实现在参数变化前后执行额外的操作。
rcl_interfaces::msg::SetParametersResult
ParametersHandler::dynamicParamsCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result; // 用于表示参数设置的结果。
  std::lock_guard<std::mutex> lock(parameters_change_mutex_);

  for (auto & pre_cb : pre_callbacks_) {     
    pre_cb();
  }

  for (auto & param : parameters) {
    const std::string & param_name = param.get_name();

    if (auto callback = get_param_callbacks_.find(param_name);
      callback != get_param_callbacks_.end())
    {
      callback->second(param);   // 找到对应的回调函数，则调用该回调函数来处理参数
    } else {
      RCLCPP_WARN(logger_, "Parameter %s not found", param_name.c_str());
    }
  }

  for (auto & post_cb : post_callbacks_) {
    post_cb();
  }

  result.successful = true;
  return result;
}


}  // namespace mppi

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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_

#include <functional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mppi
{
/**
 * @class Parameter Type enum
 */
enum class ParameterType { Dynamic, Static };

/**
 * @class mppi::ParametersHandler
 * @brief Handles getting parameters and dynamic parameter changes
 */
class ParametersHandler
{
public: // 使用 `using` 关键字定义了三个别名（typedef）：通过使用 `using` 关键字，可以为一个数据类型或函数签名定义一个更易于理解和使用的别名，从而在代码中使用这些别名来代替原始的类型或函数签名，使代码更加清晰和简洁。
  using get_param_func_t = void (const rclcpp::Parameter & param);  // get_param_func_t` 是一个函数指针类型，指向一个接受 `const rclcpp::Parameter &` 类型参数的函数，返回类型为 `void`。这个别名定义了一个函数签名，用于指定一个特定类型的函数指针。
  using post_callback_t = void ();  // `post_callback_t` 是一个函数指针类型，指向一个不接受参数且返回类型为 `void` 的函数。这个别名定义了一个函数签名，用于指定一个特定类型的函数指针。
  using pre_callback_t = void ();

  /**
    * @brief Constructor for mppi::ParametersHandler
    */
  ParametersHandler() = default;

  /**
    * @brief Constructor for mppi::ParametersHandler
    * @param parent Weak ptr to node
    */
  explicit ParametersHandler(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent);

  /**
    * @brief Starts processing dynamic parameter changes
    */
  void start();

  /**
    * @brief Dynamic parameter callback
    * @param parameter Parameter changes to process
    * @return Set Parameter Result
    */
  rcl_interfaces::msg::SetParametersResult dynamicParamsCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
    * @brief Get an object to retreive parameters
    * @param ns Namespace to get parameters within
    * @return Parameter getter object
    */
  inline auto getParamGetter(const std::string & ns);

  /**
    * @brief Set a callback to process after parameter changes
    * @param callback Callback function
    */
  template<typename T>
  void addPostCallback(T && callback);

  /**
    * @brief Set a callback to process before parameter changes
    * @param callback Callback function
    */
  template<typename T>
  void addPreCallback(T && callback);

  /**
    * @brief Set a parameter to a dynamic parameter callback
    * @param setting Parameter
    * @param name Name of parameter
    */
  template<typename T>
  void setDynamicParamCallback(T & setting, const std::string & name);

  /**
    * @brief Get mutex lock for changing parameters
    * @return Pointer to mutex
    */
  std::mutex * getLock()
  {
    return &parameters_change_mutex_;
  }

  /**
    * @brief Set a parameter to a dynamic parameter callback
    * @param name Name of parameter
    * @param callback Parameter callback
    */
  template<typename T>
  void addDynamicParamCallback(const std::string & name, T && callback);

protected:
  /**
    * @brief Gets parameter
    * @param setting Return Parameter type
    * @param name Parameter name
    * @param default_value Default parameter value
    * @param param_type Type of parameter (dynamic or static)
    */
  template<typename SettingT, typename ParamT>
  void getParam(
    SettingT & setting, const std::string & name, ParamT default_value,
    ParameterType param_type = ParameterType::Dynamic);

  /**
    * @brief Set a parameter
    * @param setting Return Parameter type
    * @param name Parameter name
    * @param node Node to set parameter via
    */
  template<typename ParamT, typename SettingT, typename NodeT>
  void setParam(SettingT & setting, const std::string & name, NodeT node) const;

  /**
    * @brief Converts parameter type to real types
    * @param parameter Parameter to convert into real type
    * @return parameter as a functional type
    */
  template<typename T>
  static auto as(const rclcpp::Parameter & parameter);

  std::mutex parameters_change_mutex_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_param_handler_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string node_name_;

  bool verbose_{false};

  std::unordered_map<std::string, std::function<get_param_func_t>>
  get_param_callbacks_;                                             // 当参数在参数服务器中发生变化时，会调用这些回调函数来更新相应的设置参数， 

  std::vector<std::function<pre_callback_t>> pre_callbacks_;   // 预回调 和 后回调 用来检查一些状态
  std::vector<std::function<post_callback_t>> post_callbacks_;
};

inline auto ParametersHandler::getParamGetter(const std::string & ns)  //  返回一个函数用于获取 相应参数，， 参数值都设置到 setting
{
  return [this, ns](
    auto & setting, const std::string & name, auto default_value,
    ParameterType param_type = ParameterType::Dynamic) {
           getParam(
             setting, ns.empty() ? name : ns + "." + name,
             std::move(default_value), param_type);
         };
}

template<typename T>
void ParametersHandler::addDynamicParamCallback(const std::string & name, T && callback)     // 设置 参数回调函数  ，
{
  get_param_callbacks_[name] = callback;
}

template<typename T>
void ParametersHandler::addPostCallback(T && callback)
{
  post_callbacks_.push_back(callback);
}

template<typename T>
void ParametersHandler::addPreCallback(T && callback)
{
  pre_callbacks_.push_back(callback);
}

template<typename SettingT, typename ParamT>
void ParametersHandler::getParam(
  SettingT & setting, const std::string & name,           // 从ros2 参数服务器中获取 参数 赋值给 setting
  ParamT default_value,
  ParameterType param_type)
{
  auto node = node_.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, name, rclcpp::ParameterValue(default_value));

  setParam<ParamT>(setting, name, node);    

  if (param_type == ParameterType::Dynamic) {
    setDynamicParamCallback(setting, name);
  }
}

template<typename ParamT, typename SettingT, typename NodeT>
void ParametersHandler::setParam(                                   // 获取参数服务器的参数值 赋值给 setting
  SettingT & setting, const std::string & name, NodeT node) const
{
  ParamT param_in{};
  node->get_parameter(name, param_in);
  setting = static_cast<SettingT>(param_in);
}

template<typename T>
void ParametersHandler::setDynamicParamCallback(T & setting, const std::string & name)  // 设置 name 对应的回调函数
{
  if (get_param_callbacks_.find(name) != get_param_callbacks_.end()) {  // 如果已经存在 名为name  的回调函数就返回 防止重复创建
    return;
  }

  // 设置一个动态参数的回调函数，当参数在参数服务器中发生变化时，会调用这个回调函数来更新相应的设置参数，
  // 并且在需要时打印日志以显示动态参数的改变情况。Lambda表达式中的 `name` 参数用于捕获外部作用域中的参数名称，以便在lambda表达式中使用该参数。

  auto callback = [this, &setting, name](const rclcpp::Parameter & param) {   // lambda表达式中 name 这个参数什么用 ???
      setting = as<T>(param);    // 首先将参数的值转换为类型 `T`，然后将其赋值给 `setting`，从而更新设置的参数值。

      if (verbose_) {
        RCLCPP_INFO(logger_, "Dynamic parameter changed: %s", std::to_string(param).c_str());
      }
    };

  addDynamicParamCallback(name, callback);

  if (verbose_) {
    RCLCPP_INFO(logger_, "Dynamic Parameter added %s", name.c_str());
  }
}

template<typename T>
auto ParametersHandler::as(const rclcpp::Parameter & parameter)
{
  if constexpr (std::is_same_v<T, bool>) {
    return parameter.as_bool();
  } else if constexpr (std::is_integral_v<T>) {
    return parameter.as_int();
  } else if constexpr (std::is_floating_point_v<T>) {
    return parameter.as_double();
  } else if constexpr (std::is_same_v<T, std::string>) {
    return parameter.as_string();
  } else if constexpr (std::is_same_v<T, std::vector<int64_t>>) {
    return parameter.as_integer_array();
  } else if constexpr (std::is_same_v<T, std::vector<double>>) {
    return parameter.as_double_array();
  } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
    return parameter.as_string_array();
  } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
    return parameter.as_bool_array();
  }
}

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_

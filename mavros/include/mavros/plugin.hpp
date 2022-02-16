/*
 * Copyright 2013,2014,2015,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS Plugin base
 * @file plugin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 *  @brief MAVROS Plugin system
 */

#pragma once

#ifndef MAVROS__PLUGIN_HPP_
#define MAVROS__PLUGIN_HPP_

#include <tuple>
#include <vector>
#include <functional>
#include <mutex>
#include <shared_mutex>     // NOLINT
#include <memory>
#include <string>
#include <unordered_map>

#include "mavconn/interface.hpp"
#include "mavros/mavros_uas.hpp"

namespace mavros
{
namespace uas
{
class UAS;
using MAV_CAP = mavlink::common::MAV_PROTOCOL_CAPABILITY;
}     // namespace uas

namespace plugin
{

using mavros::uas::UAS;
using UASPtr = std::shared_ptr<UAS>;
using r_unique_lock = std::unique_lock<std::recursive_mutex>;
using s_unique_lock = std::unique_lock<std::shared_timed_mutex>;
using s_shared_lock = std::shared_lock<std::shared_timed_mutex>;

class Filter
{
  virtual bool operator()(
    UASPtr uas, const mavlink::mavlink_message_t * cmsg,
    const mavconn::Framing framing) = 0;
};

/**
 * @brief MAVROS Plugin base class
 */
class Plugin : public std::enable_shared_from_this<Plugin>
{
private:
  explicit Plugin(const Plugin &) = delete;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(Plugin)

  //! generic message handler callback
  using HandlerCb = mavconn::MAVConnInterface::ReceivedCb;
  //! Tuple: MSG ID, MSG NAME, message type into hash_code, message handler callback
  using HandlerInfo = std::tuple<mavlink::msgid_t, const char *, size_t, HandlerCb>;
  //! Subscriptions vector
  using Subscriptions = std::vector<HandlerInfo>;

  explicit Plugin(UASPtr uas_)
  : uas(uas_), node(std::dynamic_pointer_cast<rclcpp::Node>(uas_))
  {}

  explicit Plugin(
    UASPtr uas_, const std::string & subnode,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : uas(uas_),
    // node(std::dynamic_pointer_cast<rclcpp::Node>(uas_)->create_sub_node(subnode))  // https://github.com/ros2/rclcpp/issues/731
    node(rclcpp::Node::make_shared(subnode,
      std::dynamic_pointer_cast<rclcpp::Node>(uas_)->get_fully_qualified_name(), options))
  {}

  virtual ~Plugin() = default;

  /**
   * @brief Return vector of MAVLink message subscriptions (handlers)
   */
  virtual Subscriptions get_subscriptions() = 0;

  virtual rclcpp::Node::SharedPtr get_node() const
  {
    return node;
  }

  virtual rclcpp::Logger get_logger() const
  {
    return node->get_logger();
  }

  virtual rclcpp::Clock::SharedPtr get_clock() const
  {
    return node->get_clock();
  }

protected:
  UASPtr uas;                       // uas back link
  rclcpp::Node::SharedPtr node;     // most of plugins uses sub-node

  using SetParametersResult = rcl_interfaces::msg::SetParametersResult;
  using ParameterFunctorT = std::function<void (const rclcpp::Parameter & p)>;

  std::unordered_map<std::string, ParameterFunctorT> node_watch_parameters;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr node_set_parameters_handle_ptr;

  /**
   * Make subscription to raw message.
   *
   * @param[in] id  message id
   * @param[in] fn  pointer to member function (handler)
   */
  template<class _C>
  HandlerInfo make_handler(
    const mavlink::msgid_t id, void (_C::* fn)(
      const mavlink::mavlink_message_t * msg, const mavconn::Framing framing))
  {
    auto bfn = std::bind(
      fn, std::static_pointer_cast<_C>(shared_from_this()), std::placeholders::_1,
      std::placeholders::_2);
    const auto type_hash_ = typeid(mavlink::mavlink_message_t).hash_code();

    return HandlerInfo {id, nullptr, type_hash_, bfn};
  }

  /**
   * Make subscription to message with automatic decoding.
   *
   * @param[in] fn  pointer to member function (handler)
   */
  template<class _C, class _T, class _F>
  HandlerInfo make_handler(void (_C::* fn)(const mavlink::mavlink_message_t *, _T &, _F))
  {
    static_assert(
      std::is_base_of<Filter, _F>::value,
      "Filter class should be derived from mavros::plugin::Filter");

    auto bfn = std::bind(
      fn, std::static_pointer_cast<_C>(shared_from_this()), std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3);
    const auto id = _T::MSG_ID;
    const auto name = _T::NAME;
    const auto type_hash_ = typeid(_T).hash_code();
    auto uas_ = this->uas;

    return HandlerInfo {
      id, name, type_hash_,
      [bfn, uas_](const mavlink::mavlink_message_t * msg, const mavconn::Framing framing) {
        auto filter = _F();
        if (!filter(uas_, msg, framing)) {
          return;
        }

        mavlink::MsgMap map(msg);
        _T obj;
        obj.deserialize(map);

        bfn(msg, obj, filter);
      }
    };
  }

  /**
   * Common callback called on connection change
   */
  virtual void connection_cb(bool connected)
  {
    (void)connected;
    assert(false);
  }

  /**
   * Shortcut for connection_cb() registration
   */
  void enable_connection_cb();

  /**
   * Common callback called only when capabilities change
   */
  virtual void capabilities_cb(uas::MAV_CAP capabilities)
  {
    (void)capabilities;
    assert(false);
  }

  /**
   * Shortcut for capabilities_cb() registration
   */
  void enable_capabilities_cb();

  /**
   * Default implmentation of that watch would use watch_parameters
   */
  virtual SetParametersResult node_on_set_parameters_cb(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * Shourtcut to enable default parameters watch impl
   */
  void enable_node_watch_parameters();

  /**
   * Adds parameter to watch and declares it
   */
  template<typename ParameterT>
  auto node_declare_and_watch_parameter(
    const std::string & name, ParameterFunctorT cb,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor(),
    bool ignore_override = false
  )
  {
    node_watch_parameters[name] = cb;

#ifdef USE_OLD_DECLARE_PARAMETER
    // NOTE(vooon): for Foxy:
    return node->declare_parameter(
      name,
      rclcpp::ParameterValue(), parameter_descriptor, ignore_override);
#else
    // NOTE(vooon): for Galactic:
    return node->declare_parameter<ParameterT>(name, parameter_descriptor, ignore_override);
#endif
  }

  template<typename ParameterT>
  auto node_declare_and_watch_parameter(
    const std::string & name, const ParameterT & default_value,
    ParameterFunctorT cb,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor(),
    bool ignore_override = false
  )
  {
    node_watch_parameters[name] = cb;
    return node->declare_parameter(name, default_value, parameter_descriptor, ignore_override);
  }

  //! Helper to convert ros time to mavlink time_usec field
  inline uint64_t get_time_usec(const builtin_interfaces::msg::Time & t)
  {
    return rclcpp::Time(t).nanoseconds() / 1000;
  }

  //! Helper to convert ros now time to mavlink time_usec field
  inline uint64_t get_time_usec()
  {
    return get_time_usec(node->now());
  }

  //! Helper to convert ros time to mavlink time_boot_ms field
  inline uint32_t get_time_boot_ms(const builtin_interfaces::msg::Time & t)
  {
    return rclcpp::Time(t).nanoseconds() / 1000000;
  }

  //! Helper to convert ros now time to mavlink time_boot_ms field
  inline uint32_t get_time_boot_ms()
  {
    return get_time_boot_ms(node->now());
  }
};

//! A factory class to help initialize plugin node from constructor
class PluginFactory
{
public:
  PluginFactory() = default;
  virtual ~PluginFactory() = default;

  virtual Plugin::SharedPtr create_plugin_instance(UASPtr uas) = 0;
};

//! Helper template to make plugin factories
template<typename _T>
class PluginFactoryTemplate : public PluginFactory
{
public:
  PluginFactoryTemplate() = default;
  virtual ~PluginFactoryTemplate() = default;

  Plugin::SharedPtr create_plugin_instance(UASPtr uas) override
  {
    static_assert(
      std::is_base_of<Plugin, _T>::value,
      "Plugin should be derived from mavros::plugin::Plugin");
    return std::make_shared<_T>(uas);
  }
};

}   // namespace plugin
}   // namespace mavros

#endif  // MAVROS__PLUGIN_HPP_

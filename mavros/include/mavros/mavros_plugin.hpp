/**
 * @brief MAVROS Plugin base
 * @file mavros_plugin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 *  @brief MAVROS Plugin system
 */
/*
 * Copyright 2013,2014,2015,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#ifndef MAVROS_MAVROS_PLUGIN_HPP_
#define MAVROS_MAVROS_PLUGIN_HPP_


#include <tuple>
#include <vector>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <mavconn/interface.hpp>
#include <mavros/mavros_uas.hpp>

namespace mavros
{
namespace plugin
{

using mavros::UAS;
using r_unique_lock = std::unique_lock<std::recursive_mutex>;
using s_unique_lock = std::unique_lock<std::shared_timed_mutex>;
using s_shared_lock = std::shared_lock<std::shared_timed_mutex>;

class Filter
{
  virtual static bool filter(
    UAS::SharedPtr uas, const mavlink::mavlink_message_t * cmsg,
    const mavconn::Framing framing) = 0;
}

namespace filter
{
//! AnyOk filter passes all messages with Framing::ok
class AnyOk : public Filter
{
  inline static bool filter(
    UAS::SharedPtr uas, const mavlink::mavlink_message_t * cmsg,
    const mavconn::Framing framing) override
  {
    return framing == Framing::ok;
  }
};

//! OnlySystemAndOk filter passes only messages with Framing::ok and matching target system id
class OnlySystemAndOk : public Filter
{
  inline static bool filter(
    UAS::SharedPtr uas, const mavlink::mavlink_message_t * cmsg,
    const mavconn::Framing framing) override
  {
    return framing == Framing::ok && uas->is_my_target(cmsg->sysid);
  }
};

//! OnlyComponentAndOk filter passes only messages with Framing::ok and matching target system/component ids
class OnlyComponentAndOk : public Filter
{
  inline static bool filter(
    UAS::SharedPtr uas, const mavlink::mavlink_message_t * cmsg,
    const mavconn::Framing framing) override
  {
    return framing == Framing::ok && uas->is_my_target(cmsg->sysid, cmsg->compid);
  }
};
}

/**
 * @brief MAVROS Plugin base class
 */
class Plugin : public std::enable_shared_from_this(Plugin)
{
private:
  explicit Plugin(const Plugin &) = delete;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(Plugin);

  //! generic message handler callback
  using HandlerCb = mavconn::MAVConnInterface::ReceivedCb;
  //! Tuple: MSG ID, MSG NAME, message type into hash_code, message handler callback
  using HandlerInfo = std::tuple<mavlink::msgid_t, const char *, size_t, HandlerCb>;
  //! Subscriptions vector
  using Subscriptions = std::vector<HandlerInfo>;

  explicit Plugin(UAS::SharedPtr uas_)
  : uas(uas_)
  {}

  virtual ~Plugin() = default;

  /**
   * @brief Return vector of MAVLink message subscriptions (handlers)
   */
  virtual Subscriptions get_subscriptions() = 0;

protected:
  UAS::SharedPtr uas;   // uas back link
  rclcpp::Node::SharedPtr nh;   // most of plugins uses sub-node

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

    return HandlerInfo {
      id, name, type_hash_,
      [bfn](const mavlink::mavlink_message_t * msg, const mavconn::Framing framing) {
        if (!_F::filter(uas, msg, framing)) {
          return;
        }

        mavlink::MsgMap map(msg);
        _T obj;
        obj.deserialize(map);

        bfn(msg, obj, _F());
      }
    };
  }

  /**
   * Common callback called on connection change
   */
  virtual void connection_cb(bool connected)
  {
    assert(false);
  }

  /**
   * Shortcut for connection_cb() registration
   */
  inline void enable_connection_cb()
  {
    uas->add_connection_change_handler(
      std::bind(
        &Plugin::connection_cb, this,
        std::placeholders::_1));
  }

  /**
   * Common callback called only when capabilities change
   */
  virtual void capabilities_cb(UAS::MAV_CAP capabilities)
  {
    assert(false);
  }

  /**
   * Shortcut for capabilities_cb() registration
   */
  void enable_capabilities_cb()
  {
    uas->add_capabilities_change_handler(
      std::bind(
        &Plugin::capabilities_cb, this,
        std::placeholders::_1));
  }
};

//! A factory class to help initialize plugin node from constructor
class PluginFactory
{
  PluginFactory() = default;
  virtual ~PluginFactory() = default;

  virtual Plugin::SharedPtr create_plugin_instance(UAS::SharedPtr uas) = 0;
};

//! Helper template to make plugin factories
template<typename _T>
class PluginFactoryTemplate : PluginFactory
{
  PluginFactoryTemplate() = default;
  virtual ~PluginFactoryTemplate() = default;

  Plugin::SharedPtr create_plugin_instance(UAS::SharedPtr uas) override
  {
    static_assert(
      std::is_base_of<Plugin, _T>::value,
      "Plugin should be derived from mavros::plugin::Plugin");
    return std::make_shared<_T>(uas);
  }
};

}       // namespace plugin
}       // namespace mavros

#endif  // MAVROS_MAVROS_PLUGIN_HPP_

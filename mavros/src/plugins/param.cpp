/**
 * @brief Parameter plugin
 * @file param.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <chrono>
#include <condition_variable>
#include <rcpputils/asserts.hpp>
#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>

#include <rcl_interfaces/msg/parameter_value.hpp>

#include <mavros_msgs/srv/param_set.hpp>
#include <mavros_msgs/srv/param_get.hpp>
#include <mavros_msgs/srv/param_pull.hpp>
#include <mavros_msgs/srv/param_push.hpp>
#include <mavros_msgs/msg/param.hpp>

namespace mavros
{
namespace std_plugins
{
using utils::enum_value;

/**
 * @brief Parameter storage
 *
 * Stores parameter value.
 *
 * APM uses:
 * - int8 (primary for bools)
 * - int16 ???
 * - int32 for int's
 * - real32 for float's
 *
 * PX4:
 * - int32 for int's
 * - real32 for float's
 *
 * So no reason to really use boost::any.
 * But feel free to fire an issue if your AP do not like it.
 */
class Parameter : public rclcpp::Parameter
{
public:
  using MT = mavlink::common::MAV_PARAM_TYPE;
  using PARAM_SET = mavlink::common::msg::PARAM_SET;

  uint16_t param_count;
  uint16_t param_index;

  Parameter() : rclcpp::Parameter() {}

  template<typename ValueTypeT>
  Parameter(std::string const& name, ValueTypeT value, int count, int index)
    : rclcpp::Parameter(name,value),
    param_count(count),
    param_index(index)
  {
  }

  static Parameter from_mavlink(mavlink::common::msg::PARAM_VALUE & pmsg) {
    mavlink::mavlink_param_union_t uv;
    uv.param_float = pmsg.param_value;

    auto param_name = mavlink::to_string(pmsg.param_id);

    switch (pmsg.param_type) {
      // [[[cog:
      // param_types = [ (s, 'float' if s == 'real32' else s) for s in (
      //     'int8',
      //     'int16',
      //     'int32',
      //     'real32',
      // )]
      // unsupported_types = ('uint8', 'uint16', 'uint32', 'int64', 'uint64', 'real64')
      //
      // for a, b in param_types:
      //     cog.outl("case enum_value(MT::%s):" % a.upper())
      //     cog.outl("\treturn Parameter(param_name, uv.param_%s, pmsg.param_count, pmsg.param_index);" % (b))
      // ]]]
      case enum_value(MT::INT8):
      	return Parameter(param_name, uv.param_int8, pmsg.param_count, pmsg.param_index);
      case enum_value(MT::INT16):
      	return Parameter(param_name, uv.param_int16, pmsg.param_count, pmsg.param_index);
      case enum_value(MT::INT32):
      	return Parameter(param_name, uv.param_int32, pmsg.param_count, pmsg.param_index);
      case enum_value(MT::REAL32):
      	return Parameter(param_name, uv.param_float, pmsg.param_count, pmsg.param_index);
      // [[[end]]] (checksum: 566aed8be49d83f77c0178b2608a9101)

      default:
        RCLCPP_WARN(
          rclcpp::get_logger("param"), "PM: Unsupported param %.16s (%u/%u) type: %u",
          pmsg.param_id.data(), pmsg.param_index, pmsg.param_count, pmsg.param_type);
        throw std::exception();
    }
  }

  /**
   * Variation of from_mavlink with quirks for ArduPilotMega
   */
  static Parameter from_mavlink_apm_quirk(mavlink::common::msg::PARAM_VALUE & pmsg)
  {

    auto param_name = mavlink::to_string(pmsg.param_id);

    switch (pmsg.param_type) {
      // [[[cog:
      // for a, b in param_types:
      //     btype = 'int' if 'int' in b else b
      //     cog.outl("case enum_value(MT::%s):" % a.upper())
      //     cog.outl("\treturn Parameter(param_name, pmsg.param_value, pmsg.param_count, pmsg.param_index);")
      // ]]]
      case enum_value(MT::INT8):
      	return Parameter(param_name, pmsg.param_value, pmsg.param_count, pmsg.param_index);
      case enum_value(MT::INT16):
      	return Parameter(param_name, pmsg.param_value, pmsg.param_count, pmsg.param_index);
      case enum_value(MT::INT32):
      	return Parameter(param_name, pmsg.param_value, pmsg.param_count, pmsg.param_index);
      case enum_value(MT::REAL32):
      	return Parameter(param_name, pmsg.param_value, pmsg.param_count, pmsg.param_index);
      // [[[end]]] (checksum: 268844794a29e8f7d5eb834653bd869f)

      default:
        RCLCPP_WARN(
          rclcpp::get_logger("param"), "PM: Unsupported param %.16s (%u/%u) type: %u",
          pmsg.param_id.data(), pmsg.param_index, pmsg.param_count, pmsg.param_type);
        throw std::exception();
    }
  }

  //! Make PARAM_SET message. Set target ids manually!
  PARAM_SET to_param_set()
  {
    mavlink::mavlink_param_union_t uv;
    PARAM_SET ret{};

    mavlink::set_string(ret.param_id, get_name());

    switch (get_type()) {
      // [[[cog:
      // parameter_types = (
      //     ('PARAMETER_BOOL',    'uint8', 'bool'),
      //     ('PARAMETER_INTEGER', 'int32', 'int'),
      //     ('PARAMETER_DOUBLE',  'real32', 'double'),
      // )
      //
      // for a, b, c in parameter_types:
      //     uvb = 'float' if 'real32' == b else b
      //     cog.outl("case rclcpp::ParameterType::%s:" % a)
      //     cog.outl("\tuv.param_%s = as_%s();" % (uvb, c))
      //     cog.outl("\tret.param_type = enum_value(MT::%s);" % b.upper())
      //     cog.outl("\tbreak;")
      // ]]]
      case rclcpp::ParameterType::PARAMETER_BOOL:
      	uv.param_uint8 = as_bool();
      	ret.param_type = enum_value(MT::UINT8);
      	break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
      	uv.param_int32 = as_int();
      	ret.param_type = enum_value(MT::INT32);
      	break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
      	uv.param_float = as_double();
      	ret.param_type = enum_value(MT::REAL32);
      	break;
      // [[[end]]] (checksum: bbb469e1d370511ea7c129a7f32cc95f)

      default:
        RCLCPP_WARN(rclcpp::get_logger("param"), "PR: Unsupported ParameterType: %u", get_type_name());
    }

    ret.param_value = uv.param_float;
    return ret;
  }

  //! Make PARAM_SET message. Set target ids manually!
  PARAM_SET to_param_set_apm_qurk()
  {
    PARAM_SET ret{};

    mavlink::set_string(ret.param_id, get_name());

    switch (get_type()) {
      // [[[cog:
      // for a, b, c in parameter_types:
      //     cog.outl("case rclcpp::ParameterType::%s:" % a)
      //     cog.outl("\tret.param_value = get_value<%s>();" % c)
      //     cog.outl("\tret.param_type = enum_value(MT::%s);" % b.upper())
      //     cog.outl("\tbreak;")
      // ]]]
      case rclcpp::ParameterType::PARAMETER_BOOL:
      	ret.param_value = get_value<bool>();
      	ret.param_type = enum_value(MT::UINT8);
      	break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
      	ret.param_value = get_value<int>();
      	ret.param_type = enum_value(MT::INT32);
      	break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
      	ret.param_value = get_value<double>();
      	ret.param_type = enum_value(MT::REAL32);
      	break;
      // [[[end]]] (checksum: 435bbce5eec0dc41141290086563309f)

      default:
        RCLCPP_WARN(rclcpp::get_logger("param"), "PR: Unsupported ParameterType: %u", get_type_name());
    }

    return ret;
  }

  // for debugging
  std::string to_string() const
  {
    return utils::format(
      "%s (%u/%u): %s",
      get_name().c_str(), param_index, param_count, value_to_string());
  }

  mavros_msgs::msg::Param to_msg()
  {
    mavros_msgs::msg::Param msg;

    msg.param_id = get_name();
    msg.value.integer = as_int();
    msg.value.real = as_double();
    msg.param_index = param_index;
    msg.param_count = param_count;

    return msg;
  }

  /**
   * Exclude this parameters from ~param/push
   */
  static bool check_exclude_param_id(std::string param_id)
  {
    return param_id == "SYSID_SW_MREV" ||
           param_id == "SYS_NUM_RESETS" ||
           param_id == "ARSPD_OFFSET" ||
           param_id == "GND_ABS_PRESS" ||
           param_id == "GND_ABS_PRESS2" ||
           param_id == "GND_ABS_PRESS3" ||
           param_id == "STAT_BOOTCNT" ||
           param_id == "STAT_FLTTIME" ||
           param_id == "STAT_RESET" ||
           param_id == "STAT_RUNTIME" ||
           param_id == "GND_TEMP" ||
           param_id == "CMD_TOTAL" ||
           param_id == "CMD_INDEX" ||
           param_id == "LOG_LASTFILE" ||
           param_id == "FENCE_TOTAL" ||
           param_id == "FORMAT_VERSION";
  }
};


/**
 * @brief Parameter set transaction data
 */
class ParamSetOpt
{
public:
  ParamSetOpt(Parameter & _p, size_t _rem)
  : param(_p),
    retries_remaining(_rem),
    is_timedout(false)
  {}

  Parameter param;
  size_t retries_remaining;
  bool is_timedout;
  std::mutex cond_mutex;
  std::condition_variable ack;
};


/**
 * @brief Parameter manipulation plugin
 */
class ParamPlugin : public plugin::Plugin
{
public:
  explicit ParamPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "param"),
    param_count(-1),
    param_state(PR::IDLE),
    is_timedout(false),
    RETRIES_COUNT(_RETRIES_COUNT),
    param_rx_retries(RETRIES_COUNT),
    BOOTUP_TIME_DT(BOOTUP_TIME_MS / 1000.0),
    LIST_TIMEOUT_DT(LIST_TIMEOUT_MS / 1000.0),
    PARAM_TIMEOUT_DT(PARAM_TIMEOUT_MS / 1000.0)
  {
    using namespace std::placeholders;
    pull_srv = node->create_service<mavros_msgs::srv::ParamPull>("pull", std::bind(&ParamPlugin::pull_cb, this, _1, _2));
    push_srv = node->create_service<mavros_msgs::srv::ParamPush>("push", std::bind(&ParamPlugin::push_cb, this, _1, _2));
    set_srv = node->create_service<mavros_msgs::srv::ParamSet>("set", std::bind(&ParamPlugin::set_cb, this, _1, _2));
    get_srv = node->create_service<mavros_msgs::srv::ParamGet>("get", std::bind(&ParamPlugin::get_cb, this, _1, _2));

    auto parameters_qos = rclcpp::ParametersQoS();
    param_value_pub = node->create_publisher<mavros_msgs::msg::Param>("param_value", parameters_qos);

    schedule_timer = node->create_wall_timer(BOOTUP_TIME_DT, std::bind(&ParamPlugin::schedule_cb, this));
    schedule_timer->cancel();
    timeout_timer = node->create_wall_timer(PARAM_TIMEOUT_DT, std::bind(&ParamPlugin::timeout_cb, this));
    timeout_timer->cancel();

    enable_connection_cb();
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&ParamPlugin::handle_param_value),
    };
  }

private:
  using lock_guard = std::lock_guard<std::recursive_mutex>;
  using unique_lock = std::unique_lock<std::recursive_mutex>;

  std::recursive_mutex mutex;

  rclcpp::Service<mavros_msgs::srv::ParamPull>::SharedPtr pull_srv;
  rclcpp::Service<mavros_msgs::srv::ParamPush>::SharedPtr push_srv;
  rclcpp::Service<mavros_msgs::srv::ParamSet>::SharedPtr set_srv;
  rclcpp::Service<mavros_msgs::srv::ParamGet>::SharedPtr get_srv;

  rclcpp::Publisher<mavros_msgs::msg::Param>::SharedPtr param_value_pub;

  rclcpp::TimerBase::SharedPtr schedule_timer;                            //!< for startup schedule fetch
  rclcpp::TimerBase::SharedPtr timeout_timer;                             //!< for timeout resend

  static constexpr int BOOTUP_TIME_MS = 10000;          //!< APM boot time
  static constexpr int PARAM_TIMEOUT_MS = 1000;         //!< Param wait time
  static constexpr int LIST_TIMEOUT_MS = 30000;         //!< Receive all time
  static constexpr int _RETRIES_COUNT = 3;

  const std::chrono::duration<float> BOOTUP_TIME_DT;
  const std::chrono::duration<float> LIST_TIMEOUT_DT;
  const std::chrono::duration<float> PARAM_TIMEOUT_DT;
  const int RETRIES_COUNT;

  std::unordered_map<std::string, Parameter> parameters;
  std::list<uint16_t> parameters_missing_idx;
  std::unordered_map<std::string, std::shared_ptr<ParamSetOpt>> set_parameters;
  ssize_t param_count;
  enum class PR
  {
    IDLE,
    RXLIST,
    RXPARAM,
    RXPARAM_TIMEDOUT,
    TXPARAM
  };
  PR param_state;

  size_t param_rx_retries;
  bool is_timedout;
  std::mutex list_cond_mutex;
  std::condition_variable list_receiving;

  /* -*- message handlers -*- */

  void handle_param_value(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::PARAM_VALUE & pmsg, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    lock_guard lock(mutex);

    auto param_id = mavlink::to_string(pmsg.param_id);

    // search
    auto param_it = parameters.find(param_id);
    if (param_it != parameters.end()) {
      // parameter exists
      if (uas->is_ardupilotmega()) {
        param_it->second = Parameter::from_mavlink_apm_quirk(pmsg);
      } else {
        param_it->second = Parameter::from_mavlink(pmsg);
      }

      auto & p = param_it->second;

      // check that ack required
      auto set_it = set_parameters.find(param_id);
      if (set_it != set_parameters.end()) {
        set_it->second->ack.notify_all();
      }

      auto param_value_msg = p.to_msg();
      param_value_msg.header.stamp = get_clock()->now();
      param_value_pub->publish(param_value_msg);

      RCLCPP_WARN_STREAM_EXPRESSION(
        get_logger(),
        ((p.param_index != pmsg.param_index &&
        pmsg.param_index != UINT16_MAX) ||
        p.param_count != pmsg.param_count),
        "PR: Param " << p.to_string() << " different index: " << pmsg.param_index << "/" <<
          pmsg.param_count);
      RCLCPP_DEBUG_STREAM(get_logger(), "PR: Update param " << p.to_string());
    } else {
      // insert new element
      Parameter p;

      if (uas->is_ardupilotmega()) {
        p = Parameter::from_mavlink_apm_quirk(pmsg);
      } else {
        p = Parameter::from_mavlink(pmsg);
      }

      parameters[param_id] = p;

      auto param_value_msg = p.to_msg();
      param_value_msg.header.stamp = get_clock()->now();
      param_value_pub->publish(param_value_msg);

      RCLCPP_DEBUG_STREAM(get_logger(), "PR: New param " << p.to_string());
    }

    if (param_state == PR::RXLIST || param_state == PR::RXPARAM ||
      param_state == PR::RXPARAM_TIMEDOUT)
    {

      // we received first param. setup list timeout
      if (param_state == PR::RXLIST) {
        param_count = pmsg.param_count;
        param_state = PR::RXPARAM;

        parameters_missing_idx.clear();
        if (param_count != UINT16_MAX) {
          RCLCPP_DEBUG(get_logger(), "PR: waiting %zu parameters", param_count);
          // declare that all parameters are missing
          for (uint16_t idx = 0; idx < param_count; idx++) {
            parameters_missing_idx.push_back(idx);
          }
        } else {
          RCLCPP_WARN(
            get_logger(), "PR: FCU does not know index for first element! "
            "Param list may be truncated.");
        }
      }

      // trying to avoid endless rerequest loop
      // Issue #276
      bool it_is_first_requested = parameters_missing_idx.front() == pmsg.param_index;

      // remove idx for that message
      parameters_missing_idx.remove(pmsg.param_index);

      // in receiving mode we use param_rx_retries for LIST and PARAM
      if (it_is_first_requested) {
        RCLCPP_DEBUG(
          get_logger(), "PR: got a value of a requested param idx=%u, "
          "resetting retries count", pmsg.param_index);
        param_rx_retries = RETRIES_COUNT;
      } else if (param_state == PR::RXPARAM_TIMEDOUT) {
        RCLCPP_INFO(
          get_logger(), "PR: got an unsolicited param value idx=%u, "
          "not resetting retries count %zu", pmsg.param_index, param_rx_retries);
      }

      restart_timeout_timer();

      /* index starting from 0, receivig done */
      if (parameters_missing_idx.empty()) {
        ssize_t missed = param_count - parameters.size();
        RCLCPP_INFO_EXPRESSION(get_logger(), missed == 0, "PR: parameters list received");
        RCLCPP_WARN_EXPRESSION(
          get_logger(), missed > 0,
          "PR: parameters list received, but %zd parametars are missed",
          missed);
        go_idle();
        list_receiving.notify_all();
      } else if (param_state == PR::RXPARAM_TIMEDOUT) {
        uint16_t first_miss_idx = parameters_missing_idx.front();
        RCLCPP_DEBUG(get_logger(), "PR: requesting next timed out parameter idx=%u", first_miss_idx);
        param_request_read("", first_miss_idx);
      }
    }
  }

  /* -*- low-level send function -*- */

  void param_request_list()
  {
    RCLCPP_DEBUG(get_logger(), "PR:m: request list");

    mavlink::common::msg::PARAM_REQUEST_LIST rql{};
    uas->msg_set_target(rql);

    uas->send_message(rql);
  }

  void param_request_read(std::string id, int16_t index = -1)
  {
    assert(index >= -1);

    RCLCPP_DEBUG(get_logger(), "PR:m: request '%s', idx %d", id.c_str(), index);

    mavlink::common::msg::PARAM_REQUEST_READ rqr{};
    uas->msg_set_target(rqr);
    rqr.param_index = index;

    if (index != -1) {
      mavlink::set_string(rqr.param_id, id);
    }

    uas->send_message(rqr);
  }

  void param_set(Parameter & param)
  {
    RCLCPP_DEBUG_STREAM(get_logger(), "PR:m: set param " << param.to_string());

    // GCC 4.8 can't type out lambda return
    auto ps = ([this, &param]() -> mavlink::common::msg::PARAM_SET {
        if (uas->is_ardupilotmega()) {
          return param.to_param_set_apm_qurk();
        } else {
          return param.to_param_set();
        }
      })();

    uas->msg_set_target(ps);

    uas->send_message(ps);
  }

  /* -*- mid-level functions -*- */

  void connection_cb(bool connected) override
  {
    lock_guard lock(mutex);
    if (connected) {
      schedule_pull(BOOTUP_TIME_DT);
    } else {
      schedule_timer->cancel();
    }
  }

  void schedule_pull(const std::chrono::duration<float>& dt)
  {
    schedule_timer->cancel();
    schedule_timer = node->create_wall_timer(dt,[this](){schedule_cb();});
    schedule_timer->reset();
  }

  void schedule_cb()
  {
    lock_guard lock(mutex);
    if (param_state != PR::IDLE) {
      // try later
      RCLCPP_DEBUG(get_logger(), "PR: busy, reschedule pull");
      schedule_pull(BOOTUP_TIME_DT);
    }

    RCLCPP_DEBUG(get_logger(), "PR: start scheduled pull");
    param_state = PR::RXLIST;
    param_rx_retries = RETRIES_COUNT;
    parameters.clear();

    restart_timeout_timer();
    param_request_list();
  }

  void timeout_cb()
  {
    lock_guard lock(mutex);
    if (param_state == PR::RXLIST && param_rx_retries > 0) {
      param_rx_retries--;
      RCLCPP_WARN(get_logger(), "PR: request list timeout, retries left %zu", param_rx_retries);

      restart_timeout_timer();
      param_request_list();
    } else if (param_state == PR::RXPARAM || param_state == PR::RXPARAM_TIMEDOUT) {
      if (parameters_missing_idx.empty()) {
        RCLCPP_WARN(
          get_logger(), "PR: missing list is clear, but we in RXPARAM state, "
          "maybe last rerequest fails. Params missed: %zd",
          param_count - parameters.size());
        go_idle();
        list_receiving.notify_all();
        return;
      }

      param_state = PR::RXPARAM_TIMEDOUT;
      uint16_t first_miss_idx = parameters_missing_idx.front();
      if (param_rx_retries > 0) {
        param_rx_retries--;
        RCLCPP_WARN(
          get_logger(), "PR: request param #%u timeout, retries left %zu, and %zu params still missing",
          first_miss_idx, param_rx_retries, parameters_missing_idx.size());
        restart_timeout_timer();
        param_request_read("", first_miss_idx);
      } else {
        RCLCPP_ERROR(get_logger(), "PR: request param #%u completely missing.", first_miss_idx);
        parameters_missing_idx.pop_front();
        restart_timeout_timer();
        if (!parameters_missing_idx.empty()) {
          param_rx_retries = RETRIES_COUNT;
          first_miss_idx = parameters_missing_idx.front();

          RCLCPP_WARN(
            get_logger(), "PR: %zu params still missing, trying to request next: #%u",
            parameters_missing_idx.size(), first_miss_idx);
          param_request_read("", first_miss_idx);
        }
      }
    } else if (param_state == PR::TXPARAM) {
      auto it = set_parameters.begin();
      if (it == set_parameters.end()) {
        RCLCPP_DEBUG(get_logger(), "PR: send list empty, but state TXPARAM");
        go_idle();
        return;
      }

      if (it->second->retries_remaining > 0) {
        it->second->retries_remaining--;
        RCLCPP_WARN(
          get_logger(), "PR: Resend param set for %s, retries left %zu",
          it->second->param.get_name().c_str(),
          it->second->retries_remaining);
        restart_timeout_timer();
        param_set(it->second->param);
      } else {
        RCLCPP_ERROR(
          get_logger(), "PR: Param set for %s timed out.",
          it->second->param.get_name().c_str());
        it->second->is_timedout = true;
        it->second->ack.notify_all();
      }
    } else {
      RCLCPP_DEBUG(get_logger(), "PR: timeout in IDLE!");
    }
  }

  void restart_timeout_timer()
  {
    is_timedout = false;
    timeout_timer->cancel();
    timeout_timer->reset();
  }

  void go_idle()
  {
    param_state = PR::IDLE;
    timeout_timer->cancel();
  }

  bool wait_fetch_all()
  {
    std::unique_lock<std::mutex> lock(list_cond_mutex);

    return list_receiving.wait_for(lock, LIST_TIMEOUT_DT) ==
           std::cv_status::no_timeout &&
           !is_timedout;
  }

  bool wait_param_set_ack_for(std::shared_ptr<ParamSetOpt> opt)
  {
    std::unique_lock<std::mutex> lock(opt->cond_mutex);

    return opt->ack.wait_for(
      lock,
      PARAM_TIMEOUT_DT * (RETRIES_COUNT + 2)) ==
           std::cv_status::no_timeout &&
           !opt->is_timedout;
  }

  bool send_param_set_and_wait(Parameter & param)
  {
    unique_lock lock(mutex);

    // add to waiting list
    auto opt = std::make_shared<ParamSetOpt>(param, RETRIES_COUNT);
    set_parameters[param.get_name()] = opt;

    param_state = PR::TXPARAM;
    restart_timeout_timer();
    param_set(param);

    lock.unlock();
    bool is_not_timeout = wait_param_set_ack_for(opt);
    lock.lock();

    // free opt data
    set_parameters.erase(param.get_name());

    go_idle();
    return is_not_timeout;
  }

  //! Set ROS param only if name is good
  bool rosparam_set_allowed(const Parameter & p)
  {
    if (uas->is_px4() && p.get_name() == "_HASH_CHECK") {
      auto v = p.as_int();
      RCLCPP_INFO(
        get_logger(), "PR: PX4 parameter _HASH_CHECK ignored: 0x%8x",
        static_cast<int32_t>(v));
      return false;
    }

    node->set_parameter(p);
    return true;
  }

  /* -*- ROS callbacks -*- */

  /**
   * @brief fetches all parameters from device
   * @service ~param/pull
   */
  bool pull_cb(
    mavros_msgs::srv::ParamPull::Request & req,
    mavros_msgs::srv::ParamPull::Response & res)
  {
    /*
    This may become a bit of a mess in ROS2

    unique_lock lock(mutex);

    if ((param_state == PR::IDLE && parameters.empty()) ||
      req.force_pull)
    {
      if (!req.force_pull) {
        RCLCPP_DEBUG(get_logger(), "PR: start pull");
      } else {
        RCLCPP_INFO(get_logger(), "PR: start force pull");
      }

      param_state = PR::RXLIST;
      param_rx_retries = RETRIES_COUNT;
      parameters.clear();

      schedule_timer->cancel();
      restart_timeout_timer();
      param_request_list();

      lock.unlock();
      res.success = wait_fetch_all();
    } else if (param_state == PR::RXLIST || param_state == PR::RXPARAM ||
      param_state == PR::RXPARAM_TIMEDOUT)
    {
      lock.unlock();
      res.success = wait_fetch_all();
    } else {
      lock.unlock();
      res.success = true;
    }

    lock.lock();
    res.param_received = parameters.size();

    for (auto & p : parameters) {
      lock.unlock();
      rosparam_set_allowed(p.second);
      lock.lock();
    }
    */

    return true;
  }

  /**
   * @brief push all parameter value to device
   * @service ~param/push
   */
  bool push_cb(
    mavros_msgs::srv::ParamPush::Request & req,
    mavros_msgs::srv::ParamPush::Response & res)
  {
    /*
    This may become a bit of a mess in ROS2

    auto param_map;
    if (!node->get_parameters("", param_map)) {
      return true;
    }

    int tx_count = 0;
    for (auto & param : param_map) {
      if (Parameter::check_exclude_param_id(param.first)) {
        RCLCPP_DEBUG_STREAM(get_logger(), "PR: Exclude param: " << param.first);
        continue;
      }

      unique_lock lock(mutex);
      auto param_it = parameters.find(param.first);
      if (param_it != parameters.end()) {
        // copy current state of Parameter
        auto to_send = param_it->second;

        // Update XmlRpcValue
        to_send.param_value = param.second;

        lock.unlock();
        bool set_res = send_param_set_and_wait(to_send);
        lock.lock();

        if (set_res) {
          tx_count++;
        }
      } else {
        RCLCPP_WARN_STREAM(get_logger(), "PR: Unknown rosparam: " << param.first);
      }
    }

    res.success = true;
    res.param_transfered = tx_count;
    */
    return true;
  }

  /**
   * @brief sets parameter value
   * @service ~param/set
   */
  bool set_cb(
    mavros_msgs::srv::ParamSet::Request & req,
    mavros_msgs::srv::ParamSet::Response & res)
  {
    unique_lock lock(mutex);

    if (param_state == PR::RXLIST || param_state == PR::RXPARAM ||
      param_state == PR::RXPARAM_TIMEDOUT)
    {
      RCLCPP_ERROR(get_logger(), "PR: receiving not complete");
      return false;
    }

    auto param_it = parameters.find(req.param_id);
    if (param_it != parameters.end()) {
      auto current_param = param_it->second;
      // As far as I can tell, rclcpp::Parameter is immutable
      // So here need to create a copy with the new value
      // Then when the PARAM_VALUE comes back, the internal map will be updated

      // according to ParamValue description
      bool value_is_zero = true;
      bool value_is_int = true;
      if (req.value.integer != 0) {
        value_is_zero = false;
      } else if (req.value.real != 0.0) {
        value_is_zero = false;
        value_is_int = false;
      }

      Parameter to_send;

      switch(current_param.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          // If ParameterType is bool, set to !value_is_zero
          to_send = Parameter(current_param.get_name(), !value_is_zero, current_param.param_count, current_param.param_index);
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER: {
          if( !value_is_int ) {
            RCLCPP_ERROR_STREAM(get_logger(), "PR: Incorrect parameter type for: " << req.param_id);
            res.success = false;
            return true;
          }
          to_send = Parameter(current_param.get_name(), req.value.integer, current_param.param_count, current_param.param_index);
          break;
        }
        case rclcpp::ParameterType::PARAMETER_DOUBLE: {
          if( value_is_int ) {
            RCLCPP_ERROR_STREAM(get_logger(), "PR: Incorrect parameter type for: " << req.param_id);
            res.success = false;
            return true;
          }
          to_send = Parameter(current_param.get_name(), req.value.real, current_param.param_count, current_param.param_index);
          break;
        }
      }

      lock.unlock();
      res.success = send_param_set_and_wait(to_send);
      lock.lock();

      res.value.integer = param_it->second.as_int();
      res.value.real = param_it->second.as_double();

      lock.unlock();
      rosparam_set_allowed(param_it->second);
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "PR: Unknown parameter to set: " << req.param_id);
      res.success = false;
    }

    return true;
  }

  /**
   * @brief get parameter
   * @service ~param/get
   */
  bool get_cb(
    mavros_msgs::srv::ParamGet::Request & req,
    mavros_msgs::srv::ParamGet::Response & res)
  {
    lock_guard lock(mutex);

    auto param_it = parameters.find(req.param_id);
    if (param_it != parameters.end()) {
      res.success = true;

      res.value.integer = param_it->second.as_int();
      res.value.real = param_it->second.as_double();
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "PR: Unknown parameter to get: " << req.param_id);
      res.success = false;
    }

    return true;
  }
};
}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::ParamPlugin)
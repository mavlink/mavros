/*
 * Copyright 2014,2015,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Parameter plugin
 * @file param.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <chrono>
#include <condition_variable>
#include <string>
#include <set>
#include <unordered_map>
#include <list>
#include <memory>
#include <vector>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

// #include "mavros_msgs/srv/param_set.hpp"
// #include "mavros_msgs/srv/param_get.hpp"
#include "mavros_msgs/srv/param_pull.hpp"
// #include "mavros_msgs/srv/param_push.hpp"
#include "mavros_msgs/srv/param_set_v2.hpp"
#include "mavros_msgs/msg/param_event.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT
using namespace std::chrono_literals;   // NOLINT
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
class Parameter
{
public:
  using MT = mavlink::common::MAV_PARAM_TYPE;
  using PARAM_SET = mavlink::common::msg::PARAM_SET;

  rclcpp::Time stamp;
  std::string param_id;
  rclcpp::ParameterValue param_value;
  uint16_t param_index;
  uint16_t param_count;

  explicit Parameter(
    const std::string & param_id_,
    const uint16_t param_index_ = 0,
    const uint16_t param_count_ = 0,
    const rclcpp::ParameterValue & param_value_ = rclcpp::ParameterValue())
  : stamp{},
    param_id(param_id_),
    param_value(param_value_),
    param_index(param_index_),
    param_count(param_count_)
  {
  }

  explicit Parameter(const rclcpp::Parameter & param)
  : Parameter(param.get_name(), 0, 0, param.get_parameter_value())
  {
  }

  void set_value(mavlink::common::msg::PARAM_VALUE & pmsg)
  {
    mavlink::mavlink_param_union_t uv;
    uv.param_float = pmsg.param_value;

    // #170 - copy union value to itermediate var
    int int_tmp;
    float float_tmp;

    switch (pmsg.param_type) {
      // [[[cog:
      // param_types = [ (s, 'float' if s == 'real32' else s) for s in (
      //     'int8', 'uint8',
      //     'int16', 'uint16',
      //     'int32', 'uint32',
      //     'real32',
      // )]
      // unsupported_types = ('int64', 'uint64', 'real64')
      //
      // for a, b in param_types:
      //     btype = 'int' if 'int' in b else b
      //     cog.outl(f"case enum_value(MT::{a.upper()}):")
      //     cog.outl(f"  {btype}_tmp = uv.param_{b};")
      //     cog.outl(f"  param_value = rclcpp::ParameterValue({btype}_tmp);")
      //     cog.outl(f"  break;")
      // ]]]
      case enum_value(MT::INT8):
        int_tmp = uv.param_int8;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::UINT8):
        int_tmp = uv.param_uint8;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::INT16):
        int_tmp = uv.param_int16;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::UINT16):
        int_tmp = uv.param_uint16;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::INT32):
        int_tmp = uv.param_int32;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::UINT32):
        int_tmp = uv.param_uint32;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::REAL32):
        float_tmp = uv.param_float;
        param_value = rclcpp::ParameterValue(float_tmp);
        break;
      // [[[end]]] (checksum: 9e08cfaf40fdb8644695057e6419a913)

      default:
        RCLCPP_WARN(
          get_logger(), "PR: Unsupported param %.16s (%u/%u) type: %u",
          pmsg.param_id.data(), pmsg.param_index, pmsg.param_count, pmsg.param_type);
        param_value = rclcpp::ParameterValue();
    }
  }

  /**
   * Variation of set_value with quirks for ArduPilotMega
   */
  void set_value_apm_quirk(mavlink::common::msg::PARAM_VALUE & pmsg)
  {
    int32_t int_tmp;
    float float_tmp;

    switch (pmsg.param_type) {
      // [[[cog:
      // for a, b in param_types:
      //     btype = 'int' if 'int' in b else b
      //     cog.outl(f"case enum_value(MT::{a.upper()}):")
      //     cog.outl(f"  {btype}_tmp = pmsg.param_value;")
      //     cog.outl(f"  param_value = rclcpp::ParameterValue({btype}_tmp);")
      //     cog.outl(f"  break;")
      // ]]]
      case enum_value(MT::INT8):
        int_tmp = pmsg.param_value;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::UINT8):
        int_tmp = pmsg.param_value;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::INT16):
        int_tmp = pmsg.param_value;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::UINT16):
        int_tmp = pmsg.param_value;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::INT32):
        int_tmp = pmsg.param_value;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::UINT32):
        int_tmp = pmsg.param_value;
        param_value = rclcpp::ParameterValue(int_tmp);
        break;
      case enum_value(MT::REAL32):
        float_tmp = pmsg.param_value;
        param_value = rclcpp::ParameterValue(float_tmp);
        break;
      // [[[end]]] (checksum: 5edecd0aeee6b2730b57904bc29aaff2)

      default:
        RCLCPP_WARN(
          get_logger(),
          "PR: Unsupported param %.16s (%u/%u) type: %u",
          pmsg.param_id.data(), pmsg.param_index, pmsg.param_count, pmsg.param_type);
        param_value = rclcpp::ParameterValue();
    }
  }

  //! Make PARAM_SET message. Set target ids manually!
  PARAM_SET to_param_set() const
  {
    mavlink::mavlink_param_union_t uv;
    PARAM_SET ret{};

    mavlink::set_string(ret.param_id, param_id);

    switch (param_value.get_type()) {
      // [[[cog:
      // parameter_types = (
      //     ('PARAMETER_BOOL', 'uint8', 'bool'),
      //     ('PARAMETER_INTEGER', 'int32', 'int32_t'),
      //     ('PARAMETER_DOUBLE', 'real32', 'double'),
      // )
      //
      // for a, b, c in parameter_types:
      //     uvb = 'float' if 'real32' == b else b
      //     cog.outl(f"case rclcpp::{a}:")
      //     cog.outl(f"  uv.param_{uvb} = param_value.get<{c}>();")
      //     cog.outl(f"  ret.param_type = enum_value(MT::{b.upper()});")
      //     cog.outl(f"  break;")
      // ]]]
      case rclcpp::PARAMETER_BOOL:
        uv.param_uint8 = param_value.get<bool>();
        ret.param_type = enum_value(MT::UINT8);
        break;
      case rclcpp::PARAMETER_INTEGER:
        uv.param_int32 = param_value.get<int32_t>();
        ret.param_type = enum_value(MT::INT32);
        break;
      case rclcpp::PARAMETER_DOUBLE:
        uv.param_float = param_value.get<double>();
        ret.param_type = enum_value(MT::REAL32);
        break;
      // [[[end]]] (checksum: 5ee0c1c37bca338a29b07048f7212d7d)

      default:
        RCLCPP_WARN_STREAM(
          get_logger(),
          "PR: Unsupported ParameterValue type: " << rclcpp::to_string(param_value.get_type()));
    }

    ret.param_value = uv.param_float;
    return ret;
  }

  //! Make PARAM_SET message. Set target ids manually!
  PARAM_SET to_param_set_apm_qurk() const
  {
    PARAM_SET ret{};

    mavlink::set_string(ret.param_id, param_id);

    switch (param_value.get_type()) {
      // [[[cog:
      // for a, b, c in parameter_types:
      //     cog.outl(f"case rclcpp::{a}:")
      //     cog.outl(f"  ret.param_value = param_value.get<{c}>();")
      //     cog.outl(f"  ret.param_type = enum_value(MT::{b.upper()});")
      //     cog.outl(f"  break;")
      // ]]]
      case rclcpp::PARAMETER_BOOL:
        ret.param_value = param_value.get<bool>();
        ret.param_type = enum_value(MT::UINT8);
        break;
      case rclcpp::PARAMETER_INTEGER:
        ret.param_value = param_value.get<int32_t>();
        ret.param_type = enum_value(MT::INT32);
        break;
      case rclcpp::PARAMETER_DOUBLE:
        ret.param_value = param_value.get<double>();
        ret.param_type = enum_value(MT::REAL32);
        break;
      // [[[end]]] (checksum: ba896d50b78e007407d92bd22aaceca8)

      default:
        RCLCPP_WARN_STREAM(
          get_logger(),
          "PR: Unsupported ParameterValue type: " << rclcpp::to_string(param_value.get_type()));
    }

    return ret;
  }

  // for debugging
  std::string to_string() const
  {
    auto pv = rclcpp::to_string(param_value);
    return utils::format(
      "%s (%u/%u): %s",
      param_id.c_str(), param_index, param_count, pv.c_str());
  }

  mavros_msgs::msg::ParamEvent to_msg() const
  {
    mavros_msgs::msg::ParamEvent msg{};
    msg.header.stamp = stamp;
    msg.param_id = param_id;
    msg.value = param_value.to_value_msg();
    msg.param_index = param_index;
    msg.param_count = param_count;

    return msg;
  }

  rclcpp::Parameter to_rcl() const
  {
    return {param_id, param_value};
  }

  /**
   * Exclude this parameters from ~param/push
   */
  static bool check_exclude_param_id(const std::string & param_id)
  {
    static const std::set<std::string> exclude_ids{
      "SYSID_SW_MREV",
      "SYS_NUM_RESETS",
      "ARSPD_OFFSET",
      "GND_ABS_PRESS",
      "GND_ABS_PRESS2",
      "GND_ABS_PRESS3",
      "STAT_BOOTCNT",
      "STAT_FLTTIME",
      "STAT_RESET",
      "STAT_RUNTIME",
      "GND_TEMP",
      "CMD_TOTAL",
      "CMD_INDEX",
      "LOG_LASTFILE",
      "FENCE_TOTAL",
      "FORMAT_VERSION",
      "use_sim_time"    // ROS2 Node adds this
    };

    return exclude_ids.find(param_id) != exclude_ids.end();
  }

  inline rclcpp::Logger get_logger() const
  {
    return rclcpp::get_logger("mavros.param");
  }
};


/**
 * @brief Parameter set transaction data
 */
class ParamSetOpt
{
public:
  struct Result
  {
    bool success;
    Parameter param;
  };

  ParamSetOpt(const Parameter & _p, size_t _rem)
  : param(_p),
    retries_remaining(_rem)
  {}

  Parameter param;
  std::atomic<size_t> retries_remaining;
  std::promise<Result> promise;
};


/**
 * @brief Parameter manipulation plugin
 * @plugin param
 */
class ParamPlugin : public plugin::Plugin
{
public:
  explicit ParamPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "param"),
    BOOTUP_TIME(10s),
    LIST_TIMEOUT(30s),
    PARAM_TIMEOUT(1s),
    RETRIES_COUNT(3),
    param_count(-1),
    param_state(PR::IDLE),
    is_timedout(false),
    param_rx_retries(RETRIES_COUNT)
  {
    enable_node_watch_parameters();

    auto qos = rclcpp::ParametersQoS();

    param_event_pub = node->create_publisher<mavros_msgs::msg::ParamEvent>("~/event", qos);

    pull_srv =
      node->create_service<mavros_msgs::srv::ParamPull>(
      "~/pull",
      std::bind(&ParamPlugin::pull_cb, this, _1, _2));
    // push_srv =
    //   node->create_service<mavros_msgs::srv::ParamPush>(
    //   "~/push",
    //   std::bind(&ParamPlugin::push_cb, this, _1, _2));
    set_srv =
      node->create_service<mavros_msgs::srv::ParamSetV2>(
      "~/set",
      std::bind(&ParamPlugin::set_cb, this, _1, _2));
    // get_srv =
    //   node->create_service<mavros_msgs::srv::ParamGet>(
    //   "~/get",
    //   std::bind(&ParamPlugin::get_cb, this, _1, _2));

    schedule_timer =
      node->create_wall_timer(BOOTUP_TIME, std::bind(&ParamPlugin::schedule_cb, this));
    schedule_timer->cancel();

    timeout_timer =
      node->create_wall_timer(PARAM_TIMEOUT, std::bind(&ParamPlugin::timeout_cb, this));
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
  // rclcpp::Service<mavros_msgs::srv::ParamPush>::SharedPtr push_srv;
  rclcpp::Service<mavros_msgs::srv::ParamSetV2>::SharedPtr set_srv;
  // rclcpp::Service<mavros_msgs::srv::ParamGet>::SharedPtr get_srv;

  rclcpp::Publisher<mavros_msgs::msg::ParamEvent>::SharedPtr param_event_pub;

  rclcpp::TimerBase::SharedPtr schedule_timer;   //!< for startup schedule fetch
  rclcpp::TimerBase::SharedPtr timeout_timer;   //!< for timeout resend

  const std::chrono::nanoseconds BOOTUP_TIME;
  const std::chrono::nanoseconds LIST_TIMEOUT;
  const std::chrono::nanoseconds PARAM_TIMEOUT;
  const int RETRIES_COUNT;

  enum class PR
  {
    IDLE,
    RXLIST,
    RXPARAM,
    RXPARAM_TIMEDOUT,
    TXPARAM
  };

  std::unordered_map<std::string, Parameter> parameters;
  std::list<uint16_t> parameters_missing_idx;
  std::unordered_map<std::string, std::shared_ptr<ParamSetOpt>> set_parameters;
  ssize_t param_count;
  PR param_state;

  size_t param_rx_retries;
  bool is_timedout;
  std::mutex list_cond_mutex;
  std::condition_variable list_receiving;

  /* -*- message handlers -*- */

  void handle_param_value(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::PARAM_VALUE & pmsg,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    lock_guard lock(mutex);

    auto lg = get_logger();
    auto param_id = mavlink::to_string(pmsg.param_id);

    auto update_parameter = [this, &pmsg](Parameter & p) {
        p.stamp = node->now();
        if (uas->is_ardupilotmega()) {
          p.set_value_apm_quirk(pmsg);
        } else {
          p.set_value(pmsg);
        }

        param_event_pub->publish(p.to_msg());
        rosparam_set_allowed(p);

        // check that ack required
        auto set_it = set_parameters.find(p.param_id);
        if (set_it != set_parameters.end()) {
          set_it->second->promise.set_value({true, p});
        }

        RCLCPP_WARN_STREAM_EXPRESSION(
          get_logger(), ((p.param_index != pmsg.param_index &&
          pmsg.param_index != UINT16_MAX) ||
          p.param_count != pmsg.param_count),
          "PR: Param " << p.to_string() << " different index: " << pmsg.param_index << "/" <<
            pmsg.param_count);
      };

    // search
    auto param_it = parameters.find(param_id);
    if (param_it != parameters.end()) {
      // parameter exists
      auto & p = param_it->second;

      update_parameter(p);
      RCLCPP_DEBUG_STREAM(lg, "PR: Update param " << p.to_string());

    } else {
      // insert new element
      auto pp =
        parameters.emplace(param_id, Parameter(param_id, pmsg.param_index, pmsg.param_count));
      auto & p = pp.first->second;

      update_parameter(p);
      RCLCPP_DEBUG_STREAM(lg, "PR: New param " << p.to_string());
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
          RCLCPP_DEBUG(lg, "PR: waiting %zu parameters", param_count);
          // declare that all parameters are missing
          for (uint16_t idx = 0; idx < param_count; idx++) {
            parameters_missing_idx.push_back(idx);
          }
        } else {
          RCLCPP_WARN(
            lg, "PR: FCU does not know index for first element! "
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
          lg, "PR: got a value of a requested param idx=%u, "
          "resetting retries count", pmsg.param_index);
        param_rx_retries = RETRIES_COUNT;
      } else if (param_state == PR::RXPARAM_TIMEDOUT) {
        RCLCPP_INFO(
          lg, "PR: got an unsolicited param value idx=%u, "
          "not resetting retries count %zu", pmsg.param_index, param_rx_retries);
      }

      restart_timeout_timer();

      /* index starting from 0, receivig done */
      if (parameters_missing_idx.empty()) {
        ssize_t missed = param_count - parameters.size();
        RCLCPP_INFO_EXPRESSION(lg, missed == 0, "PR: parameters list received");
        RCLCPP_WARN_EXPRESSION(
          lg,
          missed > 0, "PR: parameters list received, but %zd parametars are missed",
          missed);
        go_idle();
        list_receiving.notify_all();

      } else if (param_state == PR::RXPARAM_TIMEDOUT) {
        uint16_t first_miss_idx = parameters_missing_idx.front();
        RCLCPP_DEBUG(lg, "PR: requesting next timed out parameter idx=%u", first_miss_idx);
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

  void param_request_read(const std::string & id, int16_t index = -1)
  {
    rcpputils::require_true(index >= -1);

    RCLCPP_DEBUG(get_logger(), "PR:m: request '%s', idx %d", id.c_str(), index);

    mavlink::common::msg::PARAM_REQUEST_READ rqr{};
    uas->msg_set_target(rqr);
    rqr.param_index = index;

    if (index != -1) {
      mavlink::set_string(rqr.param_id, id);
    }

    uas->send_message(rqr);
  }

  void param_set(const Parameter & param)
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

  void clear_all_parameters()
  {
    parameters.clear();

    auto list = node->list_parameters({""}, 1);
    for (auto name : list.names) {
      node->undeclare_parameter(name);
    }
  }

  void connection_cb(bool connected) override
  {
    lock_guard lock(mutex);

    if (connected) {
      schedule_pull();
    } else {
      schedule_timer->cancel();
      clear_all_parameters();
    }
  }

  void schedule_pull()
  {
    schedule_timer->reset();
  }

  void schedule_cb()
  {
    lock_guard lock(mutex);
    schedule_timer->cancel();

    if (param_state != PR::IDLE) {
      // try later
      RCLCPP_DEBUG(get_logger(), "PR: busy, reschedule pull");
      schedule_pull();
    }

    RCLCPP_DEBUG(get_logger(), "PR: start scheduled pull");
    param_state = PR::RXLIST;
    param_rx_retries = RETRIES_COUNT;
    clear_all_parameters();

    restart_timeout_timer();
    param_request_list();
  }

  void timeout_cb()
  {
    lock_guard lock(mutex);
    timeout_timer->cancel();

    auto lg = get_logger();

    if (param_state == PR::RXLIST && param_rx_retries > 0) {
      param_rx_retries--;
      RCLCPP_WARN(lg, "PR: request list timeout, retries left %zu", param_rx_retries);

      restart_timeout_timer();
      param_request_list();

    } else if (param_state == PR::RXPARAM || param_state == PR::RXPARAM_TIMEDOUT) {
      if (parameters_missing_idx.empty()) {
        RCLCPP_WARN(
          lg, "PR: missing list is clear, but we in RXPARAM state, "
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
          lg, "PR: request param #%u timeout, retries left %zu, and %zu params still missing",
          first_miss_idx, param_rx_retries, parameters_missing_idx.size());
        restart_timeout_timer();
        param_request_read("", first_miss_idx);

      } else {
        RCLCPP_ERROR(lg, "PR: request param #%u completely missing.", first_miss_idx);
        parameters_missing_idx.pop_front();
        restart_timeout_timer();
        if (!parameters_missing_idx.empty()) {
          param_rx_retries = RETRIES_COUNT;
          first_miss_idx = parameters_missing_idx.front();

          RCLCPP_WARN(
            lg, "PR: %zu params still missing, trying to request next: #%u",
            parameters_missing_idx.size(), first_miss_idx);
          param_request_read("", first_miss_idx);
        }
      }

    } else if (param_state == PR::TXPARAM) {
      auto it = set_parameters.begin();
      if (it == set_parameters.end()) {
        RCLCPP_DEBUG(lg, "PR: send list empty, but state TXPARAM");
        go_idle();
        return;
      }

      if (it->second->retries_remaining > 0) {
        it->second->retries_remaining--;
        RCLCPP_WARN(
          lg, "PR: Resend param set for %s, retries left %zu",
          it->second->param.param_id.c_str(),
          it->second->retries_remaining.load());
        restart_timeout_timer();
        param_set(it->second->param);

      } else {
        RCLCPP_ERROR(
          lg, "PR: Param set for %s timed out.",
          it->second->param.param_id.c_str());
        it->second->promise.set_value({false, it->second->param});
      }

    } else {
      RCLCPP_DEBUG(lg, "PR: timeout in IDLE!");
    }
  }

  void restart_timeout_timer()
  {
    is_timedout = false;
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

    return list_receiving.wait_for(lock, LIST_TIMEOUT) == std::cv_status::no_timeout &&
           !is_timedout;
  }

  ParamSetOpt::Result wait_param_set_ack_for(const std::shared_ptr<ParamSetOpt> opt)
  {
    auto future = opt->promise.get_future();

    auto wres = future.wait_for(PARAM_TIMEOUT * (RETRIES_COUNT + 2));
    if (wres != std::future_status::ready) {
      return {false, opt->param};
    }

    return future.get();
  }

  ParamSetOpt::Result send_param_set_and_wait(const Parameter & param)
  {
    unique_lock lock(mutex);

    // add to waiting list
    auto opt = std::make_shared<ParamSetOpt>(param, RETRIES_COUNT);
    set_parameters[param.param_id] = opt;

    param_state = PR::TXPARAM;
    restart_timeout_timer();
    param_set(param);

    lock.unlock();
    auto res = wait_param_set_ack_for(opt);
    lock.lock();

    // free opt data
    set_parameters.erase(param.param_id);

    go_idle();
    return res;
  }

  //! Set ROS param only if name is good
  bool rosparam_set_allowed(const Parameter & p)
  {
    if (uas->is_px4() && p.param_id == "_HASH_CHECK") {
      RCLCPP_INFO(
        get_logger(), "PR: PX4 parameter _HASH_CHECK ignored: 0x%08x",
        p.param_value.get<int32_t>());
      return false;
    }

    try {
      node->declare_parameter(
        p.param_id, p.param_value,
        rcl_interfaces::msg::ParameterDescriptor(), true);
    } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) {
      node->set_parameter(p.to_rcl());
    }

    return true;
  }

  /* -*- ROS callbacks -*- */

  /**
   * @brief fetches all parameters from device
   * @service ~param/pull
   */
  void pull_cb(
    const mavros_msgs::srv::ParamPull::Request::SharedPtr req,
    mavros_msgs::srv::ParamPull::Response::SharedPtr res)
  {
    unique_lock lock(mutex);

    auto is_in_progress = [&]() -> bool {
        return
          param_state == PR::RXLIST ||
          param_state == PR::RXPARAM ||
          param_state == PR::RXPARAM_TIMEDOUT
        ;
      };

    if ((param_state == PR::IDLE && parameters.empty()) ||
      req->force_pull)
    {
      if (!req->force_pull) {
        RCLCPP_DEBUG(get_logger(), "PR: start pull");
      } else {
        RCLCPP_INFO(get_logger(), "PR: start force pull");
      }

      param_state = PR::RXLIST;
      param_rx_retries = RETRIES_COUNT;
      clear_all_parameters();

      schedule_timer->cancel();
      restart_timeout_timer();
      param_request_list();

      lock.unlock();
      res->success = wait_fetch_all();

    } else if (is_in_progress()) {
      lock.unlock();
      res->success = wait_fetch_all();

    } else {
      lock.unlock();
      res->success = true;
    }

    lock.lock();
    res->param_received = parameters.size();

    // for (auto & p : parameters) {
    //   lock.unlock();
    //   rosparam_set_allowed(p.second);
    //   lock.lock();
    // }
  }

#if 0
  /**
   * @brief push all parameter value to device
   * @service ~param/push
   */
  void push_cb(
    const mavros_msgs::ParamPush::Request::SharedPtr req,
    mavros_msgs::ParamPush::Response::SharedPtr res)
  {
    XmlRpc::XmlRpcValue param_dict;
    if (!param_nh.getParam("", param_dict)) {
      return true;
    }

    ROS_ASSERT(param_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    int tx_count = 0;
    for (auto & param : param_dict) {
      if (Parameter::check_exclude_param_id(param.first)) {
        ROS_DEBUG_STREAM_NAMED("param", "PR: Exclude param: " << param.first);
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
        ROS_WARN_STREAM_NAMED("param", "PR: Unknown rosparam: " << param.first);
      }
    }

    res.success = true;
    res.param_transfered = tx_count;

    return true;
  }
#endif

  /**
   * @brief sets parameter value
   * @service ~param/set
   */
  void set_cb(
    const mavros_msgs::srv::ParamSetV2::Request::SharedPtr req,
    mavros_msgs::srv::ParamSetV2::Response::SharedPtr res)
  {
    unique_lock lock(mutex);

    if (param_state == PR::RXLIST || param_state == PR::RXPARAM ||
      param_state == PR::RXPARAM_TIMEDOUT)
    {
      RCLCPP_ERROR(get_logger(), "PR: receiving not complete");
      throw std::runtime_error("receiving in progress");
    }

    if (Parameter::check_exclude_param_id(req->param_id) && !req->force_set) {
      RCLCPP_INFO_STREAM(get_logger(), "PR: skipping parameter: " << req->param_id);
    }

    auto param_it = parameters.find(req->param_id);
    if (param_it != parameters.end()) {
      auto to_send = param_it->second;

      to_send.param_value = rclcpp::ParameterValue(req->value);

      lock.unlock();
      auto sres = send_param_set_and_wait(to_send);
      lock.lock();

      res->value = sres.param.param_value.to_value_msg();

      // lock.unlock();
      // rosparam_set_allowed(param_it->second);

    } else if (req->force_set) {
      auto to_send = Parameter(rclcpp::Parameter(req->param_id, req->value));

      lock.unlock();
      auto sres = send_param_set_and_wait(to_send);
      lock.lock();

      res->value = sres.param.param_value.to_value_msg();

    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "PR: Unknown parameter to set: " << req->param_id);
      res->success = false;
    }
  }

#if 0
  /**
   * @brief get parameter
   * @service ~param/get
   */
  void get_cb(
    mavros_msgs::srv::ParamGet::Request::SharedPtr req,
    mavros_msgs::srv::ParamGet::Response::SharedPtr res)
  {
    lock_guard lock(mutex);

    auto param_it = parameters.find(req.param_id);
    if (param_it != parameters.end()) {
      res.success = true;

      res.value.integer = param_it->second.to_integer();
      res.value.real = param_it->second.to_real();
    } else {
      ROS_ERROR_STREAM_NAMED("param", "PR: Unknown parameter to get: " << req.param_id);
      res.success = false;
    }

    return true;
  }
#endif

  Plugin::SetParametersResult node_on_set_parameters_cb(
    const std::vector<rclcpp::Parameter> & set_parameters) override
  {
    SetParametersResult result;

    // XXX: test me!

    result.successful = true;

    auto is_changed = [this](const rclcpp::Parameter & p) {
        unique_lock lock(mutex);

        auto param_it = parameters.find(p.get_name());
        if (param_it == parameters.end()) {
          return true;
        }

        return param_it->second.param_value != p.get_parameter_value();
      };

    for (auto & p : set_parameters) {
      auto req = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
      auto res = std::make_shared<mavros_msgs::srv::ParamSetV2::Response>();

      req->param_id = p.get_name();
      req->value = p.get_parameter_value().to_value_msg();

      if (!is_changed(p)) {
        continue;
      }

      try {
        set_cb(req, res);
      } catch (std::exception & ex) {
        RCLCPP_ERROR_STREAM(get_logger(), "PR: error: " << ex.what());
      }

      if (!res->success) {
        RCLCPP_ERROR(get_logger(), "PR: Failed to set parameter: %s", req->param_id.c_str());
        // result.successful = false;
      }
    }

    return result;
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::ParamPlugin)

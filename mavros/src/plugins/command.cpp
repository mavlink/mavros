/**
 * @brief Command plugin
 * @file command.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <chrono>
#include <condition_variable>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Empty.h>

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandInt.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandTriggerControl.h>

namespace mavplugin {
class CommandTransaction {
public:
  std::mutex cond_mutex;
  std::condition_variable ack;
  uint16_t expected_command;
  uint8_t result;

  explicit CommandTransaction(uint16_t command) :
    ack(),
    expected_command(command),
    result(MAV_RESULT_FAILED)
  { }
};

/**
 * @brief Command plugin.
 *
 * Send any command via COMMAND_LONG
 */
class CommandPlugin : public MavRosPlugin {
public:
  CommandPlugin() :
    uas(nullptr),
    cmd_nh("~cmd"),
    use_comp_id_system_control(false),
    ACK_TIMEOUT_DT(ACK_TIMEOUT_MS / 1000.0)
  { };

  void initialize(UAS &uas_)
  {
    uas = &uas_;

    cmd_nh.param("use_comp_id_system_control", use_comp_id_system_control, false);

    command_long_srv = cmd_nh.advertiseService("command", &CommandPlugin::command_long_cb, this);
    command_int_srv = cmd_nh.advertiseService("command_int", &CommandPlugin::command_int_cb, this);
    arming_srv = cmd_nh.advertiseService("arming", &CommandPlugin::arming_cb, this);
    set_home_srv = cmd_nh.advertiseService("set_home", &CommandPlugin::set_home_cb, this);
    takeoff_srv = cmd_nh.advertiseService("takeoff", &CommandPlugin::takeoff_cb, this);
    land_srv = cmd_nh.advertiseService("land", &CommandPlugin::land_cb, this);
    trigger_srv = cmd_nh.advertiseService("trigger_control", &CommandPlugin::trigger_control_cb, this);
    deploy_payload_srv = cmd_nh.advertiseService("deploy_payload", &CommandPlugin::deploy_payload_cb, this);
  }

  const message_map get_rx_handlers() {
    return {
             MESSAGE_HANDLER(MAVLINK_MSG_ID_COMMAND_ACK, &CommandPlugin::handle_command_ack)
    };
  }

private:
  std::recursive_mutex mutex;
  UAS *uas;

  ros::NodeHandle cmd_nh;
  ros::ServiceServer command_long_srv;
  ros::ServiceServer command_int_srv;
  ros::ServiceServer arming_srv;
  ros::ServiceServer set_home_srv;
  ros::ServiceServer takeoff_srv;
  ros::ServiceServer land_srv;
  ros::ServiceServer trigger_srv;
  ros::ServiceServer deploy_payload_srv;

  bool use_comp_id_system_control;

  std::list<CommandTransaction *> ack_waiting_list;
  static constexpr int ACK_TIMEOUT_MS = 5000;

  const ros::Duration ACK_TIMEOUT_DT;

  /* -*- message handlers -*- */

  void handle_command_ack(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(msg, &ack);

    lock_guard lock(mutex);
    for (auto it = ack_waiting_list.cbegin();
        it != ack_waiting_list.cend(); it++)
      if ((*it)->expected_command == ack.command) {
        (*it)->result = ack.result;
        (*it)->ack.notify_all();
        return;
      }

    ROS_WARN_THROTTLE_NAMED(10, "cmd", "CMD: Unexpected command %u, result %u",
        ack.command, ack.result);
  }

  /* -*- mid-level functions -*- */

  bool wait_ack_for(CommandTransaction *tr) {
    std::unique_lock<std::mutex> lock(tr->cond_mutex);

    return tr->ack.wait_for(lock, std::chrono::nanoseconds(ACK_TIMEOUT_DT.toNSec()))
           == std::cv_status::no_timeout;
  }

  /**
   * Common function for command service callbacks.
   *
   * NOTE: success is bool in messages, but has unsigned char type in C++
   */
  bool send_command_long_and_wait(bool broadcast,
      uint16_t command, uint8_t confirmation,
      float param1, float param2,
      float param3, float param4,
      float param5, float param6,
      float param7,
      unsigned char &success, uint8_t &result)
  {
    unique_lock lock(mutex);

    /* check transactions */
    for (auto it = ack_waiting_list.cbegin();
        it != ack_waiting_list.cend(); it++)
      if ((*it)->expected_command == command) {
        ROS_WARN_THROTTLE_NAMED(10, "cmd", "CMD: Command %u alredy in progress", command);
        return false;
      }

    /**
     * @note APM always send COMMAND_ACK, while PX4 never.
     * Don't expect any ACK in broadcast mode.
     */
    bool is_ack_required = (confirmation != 0 || uas->is_ardupilotmega()) && !uas->is_px4();
    if (is_ack_required && !broadcast)
      ack_waiting_list.push_back(new CommandTransaction(command));

    command_long(broadcast,
        command, confirmation,
        param1, param2,
        param3, param4,
        param5, param6,
        param7);

    if (is_ack_required && !broadcast) {
      auto it = ack_waiting_list.begin();
      for (; it != ack_waiting_list.end(); it++)
        if ((*it)->expected_command == command)
          break;

      if (it == ack_waiting_list.end()) {
        ROS_ERROR_NAMED("cmd", "CMD: CommandTransaction not found for %u", command);
        return false;
      }

      lock.unlock();
      bool is_not_timeout = wait_ack_for(*it);
      lock.lock();

      success = is_not_timeout && (*it)->result == MAV_RESULT_ACCEPTED;
      result = (*it)->result;

      delete *it;
      ack_waiting_list.erase(it);
    }
    else {
      success = true;
      result = MAV_RESULT_ACCEPTED;
    }

    return true;
  }

  /**
   * Common function for COMMAND_INT service callbacks.
   */
  bool send_command_int(bool broadcast,
      uint8_t frame, uint16_t command,
      uint8_t current, uint8_t autocontinue,
      float param1, float param2,
      float param3, float param4,
      int32_t x, int32_t y,
      float z,
      unsigned char &success) {
    /* Note: seems that COMMAND_INT don't produce COMMAND_ACK
     * so wait don't needed.
     */
    command_int(broadcast,
        frame, command, current, autocontinue,
        param1, param2,
        param3, param4,
        x, y, z);

    success = true;
    return true;
  }

  /* -*- low-level send -*- */

  void command_long(bool broadcast,
      uint16_t command, uint8_t confirmation,
      float param1, float param2,
      float param3, float param4,
      float param5, float param6,
      float param7)
  {
    mavlink_message_t msg;
    const uint8_t tgt_sys_id = (broadcast) ? 0 : uas->get_tgt_system();
    const uint8_t tgt_comp_id = (broadcast) ? 0 :
      (use_comp_id_system_control) ?
        MAV_COMP_ID_SYSTEM_CONTROL : uas->get_tgt_component();
    const uint8_t confirmation_fixed = (broadcast) ? 0 : confirmation;

    mavlink_msg_command_long_pack_chan(UAS_PACK_CHAN(uas), &msg,
        tgt_sys_id,
        tgt_comp_id,
        command,
        confirmation_fixed,
        param1, param2,
        param3, param4,
        param5, param6,
        param7);
    UAS_FCU(uas)->send_message(&msg);
  }

  void command_int(bool broadcast,
      uint8_t frame, uint16_t command,
      uint8_t current, uint8_t autocontinue,
      float param1, float param2,
      float param3, float param4,
      int32_t x, int32_t y,
      float z)
  {
    mavlink_message_t msg;
    const uint8_t tgt_sys_id = (broadcast) ? 0 : uas->get_tgt_system();
    const uint8_t tgt_comp_id = (broadcast) ? 0 :
      (use_comp_id_system_control) ?
        MAV_COMP_ID_SYSTEM_CONTROL : uas->get_tgt_component();

    mavlink_msg_command_int_pack_chan(UAS_PACK_CHAN(uas), &msg,
        tgt_sys_id,
        tgt_comp_id,
        frame,
        command,
        current,
        autocontinue,
        param1, param2,
        param3, param4,
        x, y, z);
    UAS_FCU(uas)->send_message(&msg);
  }

  /* -*- callbacks -*- */

  bool command_long_cb(mavros_msgs::CommandLong::Request &req,
      mavros_msgs::CommandLong::Response &res) {
    return send_command_long_and_wait(req.broadcast,
        req.command, req.confirmation,
        req.param1, req.param2,
        req.param3, req.param4,
        req.param5, req.param6,
        req.param7,
        res.success, res.result);
  }

  bool command_int_cb(mavros_msgs::CommandInt::Request &req,
      mavros_msgs::CommandInt::Response &res) {
    return send_command_int(req.broadcast,
        req.frame, req.command,
        req.current, req.autocontinue,
        req.param1, req.param2,
        req.param3, req.param4,
        req.x, req.y, req.z,
        res.success);
  }

  bool arming_cb(mavros_msgs::CommandBool::Request &req,
      mavros_msgs::CommandBool::Response &res) {
    return send_command_long_and_wait(false,
        MAV_CMD_COMPONENT_ARM_DISARM, 1,
        (req.value) ? 1.0 : 0.0,
        0, 0, 0, 0, 0, 0,
        res.success, res.result);
  }

  bool set_home_cb(mavros_msgs::CommandHome::Request &req,
      mavros_msgs::CommandHome::Response &res) {
    return send_command_long_and_wait(false,
        MAV_CMD_DO_SET_HOME, 1,
        (req.current_gps) ? 1.0 : 0.0,
        0, 0, 0, req.latitude, req.longitude, req.altitude,
        res.success, res.result);
  }

  bool takeoff_cb(mavros_msgs::CommandTOL::Request &req,
      mavros_msgs::CommandTOL::Response &res) {
    return send_command_long_and_wait(false,
        MAV_CMD_NAV_TAKEOFF, 1,
        req.min_pitch,
        0, 0,
        req.yaw,
        req.latitude, req.longitude, req.altitude,
        res.success, res.result);
  }

  bool land_cb(mavros_msgs::CommandTOL::Request &req,
      mavros_msgs::CommandTOL::Response &res) {
    return send_command_long_and_wait(false,
        MAV_CMD_NAV_LAND, 1,
        0, 0, 0,
        req.yaw,
        req.latitude, req.longitude, req.altitude,
        res.success, res.result);
  }

  bool trigger_control_cb(mavros_msgs::CommandTriggerControl::Request &req,
      mavros_msgs::CommandTriggerControl::Response &res) {
    return send_command_long_and_wait(false,
        MAV_CMD_DO_TRIGGER_CONTROL, 1,
        (req.trigger_enable)? 1.0 : 0.0,
        req.integration_time,
        0, 0, 0, 0, 0,
        res.success, res.result);
  }

  bool deploy_payload_cb(std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res) {
    unsigned char success = false;
    uint8_t result = 0;

    send_command_long_and_wait(false,
        MAV_CMD_PAYLOAD_CONTROL_DEPLOY, 1,
        0, 0, 0, 0, 0, 0, 0,
        success, result);

    return success;
  }
};
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::CommandPlugin, mavplugin::MavRosPlugin)


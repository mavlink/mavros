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
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <mavros/CommandLong.h>
#include <mavros/CommandBool.h>
#include <mavros/CommandMode.h>
#include <mavros/CommandHome.h>

namespace mavplugin {

class CommandTransaction {
public:
	boost::condition_variable ack;
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
	CommandPlugin()
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;

		cmd_nh = ros::NodeHandle(nh, "cmd");
		command_long_srv = cmd_nh.advertiseService("command", &CommandPlugin::command_long_cb, this);
		arming_srv = cmd_nh.advertiseService("arming", &CommandPlugin::arming_cb, this);
		set_mode_srv = cmd_nh.advertiseService("set_mode", &CommandPlugin::set_mode_cb, this);
		set_home_srv = cmd_nh.advertiseService("set_home", &CommandPlugin::set_home_cb, this);
	}

	std::string const get_name() const {
		return "Command";
	}

	std::vector<uint8_t> const get_supported_messages() const {
		return {
			MAVLINK_MSG_ID_COMMAND_ACK
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_command_ack_t ack;
		mavlink_msg_command_ack_decode(msg, &ack);

		boost::recursive_mutex::scoped_lock lock(mutex);
		for (auto it = ack_waiting_list.cbegin();
				it != ack_waiting_list.cend(); it++)
			if ((*it)->expected_command == ack.command) {
				(*it)->result = ack.result;
				(*it)->ack.notify_all();
				return;
			}

		ROS_WARN_THROTTLE_NAMED(10, "cmd", "Unexpected command %u, result %u",
			ack.command, ack.result);
	}

private:
	boost::recursive_mutex mutex;
	UAS *uas;

	ros::NodeHandle cmd_nh;
	ros::ServiceServer command_long_srv;
	ros::ServiceServer arming_srv;
	ros::ServiceServer set_mode_srv;
	ros::ServiceServer set_home_srv;

	std::list<CommandTransaction *> ack_waiting_list;
	static constexpr int ACK_TIMEOUT_MS = 5000;

	/* -*- mid-level functions -*- */

	bool wait_ack_for(CommandTransaction *tr) {
		boost::mutex cond_mutex;
		boost::unique_lock<boost::mutex> lock(cond_mutex);

		return tr->ack.timed_wait(lock, boost::posix_time::milliseconds(ACK_TIMEOUT_MS));
	}

	/**
	 * Common function for command service callbacks.
	 *
	 * NOTE: success is bool in messages, but has unsigned char type in C++
	 */
	bool send_command_long_and_wait(uint16_t command, uint8_t confirmation,
			float param1, float param2,
			float param3, float param4,
			float param5, float param6,
			float param7,
			unsigned char &success, uint8_t &result) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		/* check transactions */
		for (auto it = ack_waiting_list.cbegin();
				it != ack_waiting_list.cend(); it++)
			if ((*it)->expected_command == command) {
				ROS_WARN_THROTTLE_NAMED(10, "cmd", "Command %u alredy in progress", command);
				return false;
			}

		bool is_ack_required = confirmation != 0 || uas->is_ardupilotmega();
		if (is_ack_required)
			ack_waiting_list.push_back(new CommandTransaction(command));

		command_long(command, confirmation,
				param1, param2,
				param3, param4,
				param5, param6,
				param7);

		if (is_ack_required) {
			auto it = ack_waiting_list.begin();
			for (; it != ack_waiting_list.end(); it++)
				if ((*it)->expected_command == command)
					break;

			if (it == ack_waiting_list.end()) {
				ROS_ERROR_NAMED("cmd", "CommandTransaction not found for %u", command);
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

	/* -*- low-level send -*- */

	void command_long(uint16_t command, uint8_t confirmation,
			float param1, float param2,
			float param3, float param4,
			float param5, float param6,
			float param7) {
		mavlink_message_t msg;

		mavlink_msg_command_long_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas),
				command,
				confirmation,
				param1,
				param2,
				param3,
				param4,
				param5,
				param6,
				param7);
		uas->mav_link->send_message(&msg);
	}

	/* -*- callbacks -*- */

	bool command_long_cb(mavros::CommandLong::Request &req,
			mavros::CommandLong::Response &res) {

		return send_command_long_and_wait(req.command, req.confirmation,
				req.param1, req.param2,
				req.param3, req.param4,
				req.param5, req.param6,
				req.param7,
				res.success, res.result);
	}

	bool arming_cb(mavros::CommandBool::Request &req,
			mavros::CommandBool::Response &res) {

		return send_command_long_and_wait(MAV_CMD_COMPONENT_ARM_DISARM, 1,
				(req.value)? 1.0 : 0.0,
				0, 0, 0, 0, 0, 0,
				res.success, res.result);
	}

	bool set_mode_cb(mavros::CommandMode::Request &req,
			mavros::CommandMode::Response &res) {

		if (req.mode > 256) {
			ROS_ERROR_NAMED("cmd", "Unknown mode %u", req.mode);
			return false;
		}

		/* TODO: Add FCU-specific mode set
		 * like APM LAND,TAKEOFF and other
		 */

		return send_command_long_and_wait(MAV_CMD_DO_SET_MODE, 1,
				req.mode,
				0, 0, 0, 0, 0, 0,
				res.success, res.result);
	}

	bool set_home_cb(mavros::CommandHome::Request &req,
			mavros::CommandHome::Response &res) {

		return send_command_long_and_wait(MAV_CMD_DO_SET_HOME, 1,
				(req.current_gps)? 1.0 : 0.0,
				0, 0, 0, req.latitude, req.longitude, req.altitude,
				res.success, res.result);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::CommandPlugin, mavplugin::MavRosPlugin)


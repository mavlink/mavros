/**
 * @brief Command plugin
 * @file command.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
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

namespace mavplugin {

class CommandPlugin : public MavRosPlugin {
public:
	CommandPlugin() :
		expected_command(0),
		last_result(0),
		in_transaction(false)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;

		cmd_nh = ros::NodeHandle(nh, "cmd");
		command_long_srv = cmd_nh.advertiseService("command", &CommandPlugin::command_long_cb, this);
	}

	std::string get_name() {
		return "Command";
	}

	std::vector<uint8_t> get_supported_messages() {
		return {
			MAVLINK_MSG_ID_COMMAND_ACK
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_command_ack_t ack;
		mavlink_msg_command_ack_decode(msg, &ack);

		boost::recursive_mutex::scoped_lock lock(mutex);
		if (ack.command != expected_command) {
			ROS_WARN_THROTTLE_NAMED(10, "cmd", "Unexpected command %u (not %u), result %u",
					ack.command, expected_command, ack.result);
			return;
		}

		last_result = ack.result;
		command_ack.notify_all();
	}

private:
	boost::recursive_mutex mutex;
	UAS *uas;

	ros::NodeHandle cmd_nh;
	ros::ServiceServer command_long_srv;

	boost::condition_variable command_ack;
	uint16_t expected_command;
	uint8_t last_result;
	bool in_transaction;

	const int ACK_TIMEOUT_MS = 5000;

	/* -*- mid-level functions -*- */

	bool wait_ack() {
		boost::mutex cond_mutex;
		boost::unique_lock<boost::mutex> lock(cond_mutex);

		return command_ack.timed_wait(lock, boost::posix_time::milliseconds(ACK_TIMEOUT_MS));
	}

	/* -*- low-level send -*- */

	void command_long(uint16_t command, uint8_t confirmation,
			float param1, float param2,
			float param3, float param4,
			float param5, float param6,
			float param7) {
		mavlink_message_t msg;

		mavlink_msg_command_long_pack(0, 0, &msg,
				uas->get_tgt_system(),
				uas->get_tgt_component(),
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
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (in_transaction)
			return false;	// another transaction in progress

		in_transaction = true;
		expected_command = req.command;
		last_result = MAV_RESULT_FAILED;

		lock.unlock();
		command_long(req.command, req.confirmation,
				req.param1, req.param2,
				req.param3, req.param4,
				req.param5, req.param6,
				req.param7);
		bool isto = wait_ack();
		lock.lock();

		in_transaction = false;
		expected_command = 0;
		res.success = isto && last_result == MAV_RESULT_ACCEPTED;
		res.result = last_result;

		return true;
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::CommandPlugin, mavplugin::MavRosPlugin)


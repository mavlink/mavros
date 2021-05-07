/**
 * @brief Parachute plugin
 * @file parachute.cpp
 * @author Charlie Burge <charlieburge@yahoo.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2021 Charlie Burge.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <std_srvs/Trigger.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/Parachute.h>
#include <mavros_msgs/ParachuteCancel.h>

namespace mavros {
namespace extra_plugins {
using utils::enum_value;
using mavlink::common::MAV_CMD;
using mavlink::common::PARACHUTE_ACTION;

/**
 * @brief Parachute plugin
 *
 * This plugin is intended to be the interface for the parachute system.
 */
class ParachutePlugin : public plugin::PluginBase {
public:
	ParachutePlugin() : PluginBase(),
		pc_nh("~parachute")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		auto_chute_pub = pc_nh.advertise<mavros_msgs::Parachute>("status", 1, true);

		enable_chute = pc_nh.advertiseService("enable", &ParachutePlugin::handle_chute_enable, this);
		disable_chute = pc_nh.advertiseService("disable", &ParachutePlugin::handle_chute_disable, this);
		auto_chute_cancel = pc_nh.advertiseService("cancel", &ParachutePlugin::handle_chute_cancel, this);
		deploy_chute = pc_nh.advertiseService("release", &ParachutePlugin::handle_chute_deploy, this);
	}

	Subscriptions get_subscriptions() override
	{
		return {
					make_handler(&ParachutePlugin::handle_autochute)
		};
	}

private:
	ros::NodeHandle pc_nh;

	ros::Publisher chute_pub;

	ros::ServiceServer chute_cancel;
	ros::ServiceServer deploy_chute;
	ros::ServiceServer enable_chute;
	ros::ServiceServer disable_chute;

	//auto-parachute release reason
	enum RELEASE_REASON {
		SINK_RATE = 0,
		ACCEL_FALLING = 1,
		CONTROL_LOSS = 2,
		MISSION_ITEM = 3,
		MANUAL = 4,
	};

	void handle_autochute(const mavlink::mavlink_message_t *msg, mavlink::common::msg::COMMAND_LONG &cmd){
		if (cmd.command == enum_value(MAV_CMD::USER_1)){
			
			auto chute_msg = boost::make_shared<mavros_msgs::Parachute>();

			uint8_t _reasons = cmd.param1; 
			chute_msg->SINK_RATE = (_reasons & (1U << RELEASE_REASON::SINK_RATE));
			chute_msg->ACCEL_FALLING = (_reasons & (1U << RELEASE_REASON::ACCEL_FALLING));
			chute_msg->CONTROL_LOSS = (_reasons & (1U << RELEASE_REASON::CONTROL_LOSS));
			chute_msg->MISSION_ITEM = (_reasons & (1U << RELEASE_REASON::MISSION_ITEM));
			chute_msg->MANUAL = (_reasons & (1U << RELEASE_REASON::MANUAL));
			
			chute_msg->time_to_release = cmd.param2;
			chute_msg->standby = cmd.param3;
			chute_msg->enabled = (cmd.param4 > 0) ? true : false;
			chute_msg->released = cmd.param5;

			auto_chute_pub.publish(chute_msg);
		};
	};

	bool handle_chute_cancel(mavros_msgs::ParachuteCancel::Request &req,
			mavros_msgs::ParachuteCancel::Response &res)
	{
		try {
			ros::NodeHandle pnh("~");
			auto client = pnh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

			mavros_msgs::CommandLong cmd{};

			uint8_t reason_reset_mask = 0;

			reason_reset_mask |= (req.SINK_RATE) ? (1U << RELEASE_REASON::SINK_RATE) : 0;
			reason_reset_mask |= (req.ACCEL_FALLING) ? (1U << RELEASE_REASON::ACCEL_FALLING) : 0;
			reason_reset_mask |= (req.CONTROL_LOSS) ? (1U << RELEASE_REASON::CONTROL_LOSS) : 0;
			reason_reset_mask |= (req.MISSION_ITEM) ? (1U << RELEASE_REASON::MISSION_ITEM) : 0;
			reason_reset_mask |= (req.MANUAL) ? (1U << RELEASE_REASON::MANUAL) : 0;

			cmd.request.command = enum_value(MAV_CMD::USER_1);
			cmd.request.confirmation = 1; //get confirmation
			cmd.request.param1 = reason_reset_mask;

			res.success = client.call(cmd);
			res.success = cmd.response.success;
		}
		catch (ros::InvalidNameException &ex) {
			ROS_ERROR_NAMED("parachute", "PCH: %s", ex.what());
		}

		return true;
	}

	bool handle_chute_enable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
	{
		try {
			ros::NodeHandle pnh("~");
			auto client = pnh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

			mavros_msgs::CommandLong cmd{};

			cmd.request.command = enum_value(MAV_CMD::DO_PARACHUTE);
			cmd.request.confirmation = 1; //get confirmation
			cmd.request.param1 = enum_value(PARACHUTE_ACTION::ENABLE);

			res.success = client.call(cmd);
			res.success = cmd.response.success;
		}
		catch (ros::InvalidNameException &ex) {
			ROS_ERROR_NAMED("parachute", "PCH: %s", ex.what());
		}

		return true;
	}

	bool handle_chute_disable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
	{
		try {
			ros::NodeHandle pnh("~");
			auto client = pnh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

			mavros_msgs::CommandLong cmd{};

			cmd.request.command = enum_value(MAV_CMD::DO_PARACHUTE);
			cmd.request.confirmation = 1; //get confirmation
			cmd.request.param1 = enum_value(PARACHUTE_ACTION::DISABLE);
		catch (ros::InvalidNameException &ex) {
	}

	bool handle_chute_deploy(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
	{
		try {
			ros::NodeHandle pnh("~");
			auto client = pnh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

			mavros_msgs::CommandLong cmd{};

			cmd.request.command = enum_value(MAV_CMD::DO_PARACHUTE);
			cmd.request.confirmation = 1; //get confirmation
			cmd.request.param1 = enum_value(PARACHUTE_ACTION::RELEASE);

			res.success = client.call(cmd);
			res.success = cmd.response.success;
		}
		catch (ros::InvalidNameException &ex) {
			ROS_ERROR_NAMED("parachute", "PCH: %s", ex.what());
		}

		return true;
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ParachutePlugin, mavros::plugin::PluginBase)
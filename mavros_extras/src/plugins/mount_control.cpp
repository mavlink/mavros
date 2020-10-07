/**
 * @brief Mouny Control plugin
 * @file mount_control.cpp
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2019 Jaeyoung Lim.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/MountControl.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/MountConfigure.h>

namespace mavros {
namespace extra_plugins {

//! Mavlink enumerations
using mavlink::common::MAV_MOUNT_MODE;
using mavlink::common::MAV_CMD;
using utils::enum_value;

/**
 * @brief Mount Control plugin
 *
 * Publishes Mission commands to control the camera or antenna mount.
 * @see command_cb()
 */
class MountControlPlugin : public plugin::PluginBase {
public:
	MountControlPlugin() : PluginBase(),
	nh("~"),
	mount_nh("~mount_control")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		command_sub = mount_nh.subscribe("command", 10, &MountControlPlugin::command_cb, this);
		mount_orientation_pub = mount_nh.advertise<geometry_msgs::Quaternion>("orientation", 10);
		configure_srv = mount_nh.advertiseService("configure", &MountControlPlugin::mount_configure_cb, this);

	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&MountControlPlugin::handle_mount_orientation)
		};
	}

private:
	ros::NodeHandle nh;
	ros::NodeHandle mount_nh;
	ros::Subscriber command_sub;
	ros::Publisher mount_orientation_pub;
 	ros::ServiceServer configure_srv;

	/**
	 * @brief Publish the mount orientation
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#MOUNT_ORIENTATION
	 * @param msg   the mavlink message
	 * @param mo	received MountOrientation msg
	 */
	void handle_mount_orientation(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MOUNT_ORIENTATION &mo)
	{
		auto q = ftf::quaternion_from_rpy(Eigen::Vector3d(mo.roll, mo.pitch, mo.yaw) * M_PI / 180.0);
		geometry_msgs::Quaternion quaternion_msg;
		tf::quaternionEigenToMsg(q, quaternion_msg);
		mount_orientation_pub.publish(quaternion_msg);
	}

	/**
	 * @brief Send mount control commands to vehicle
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL
	 * @param req	received MountControl msg
	 */
	void command_cb(const mavros_msgs::MountControl::ConstPtr &req)
	{
		mavlink::common::msg::COMMAND_LONG cmd {};

		cmd.target_system = m_uas->get_tgt_system();
		cmd.target_component = m_uas->get_tgt_component();
		cmd.command = enum_value(MAV_CMD::DO_MOUNT_CONTROL);
		cmd.param1 = req->pitch;
		cmd.param2 = req->roll;
		cmd.param3 = req->yaw;
		cmd.param4 = req->altitude; // 
		cmd.param5 = req->latitude; // lattitude in degrees * 1E7
		cmd.param6 = req->longitude; // longitude in degrees * 1E7
		cmd.param7 = req->mode; // MAV_MOUNT_MODE

		UAS_FCU(m_uas)->send_message_ignore_drop(cmd);
	}

    bool mount_configure_cb(mavros_msgs::MountConfigure::Request &req,
            mavros_msgs::MountConfigure::Response &res)
    {
        using mavlink::common::MAV_CMD;

        try {
            auto client = nh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

            mavros_msgs::CommandLong cmd{};

            cmd.request.broadcast = false;
            cmd.request.command = enum_value(MAV_CMD::DO_MOUNT_CONFIGURE);
            cmd.request.confirmation = false;
            cmd.request.param1 = req.mode;
            cmd.request.param2 = req.stabilize_roll;
            cmd.request.param3 = req.stabilize_pitch;
            cmd.request.param4 = req.stabilize_yaw;
            cmd.request.param5 = req.roll_input;
            cmd.request.param6 = req.pitch_input;
            cmd.request.param7 = req.yaw_input;

            ROS_DEBUG_NAMED("mount", "MountConfigure: Request mode %u ", req.mode);
            res.success = client.call(cmd);
        }
        catch (ros::InvalidNameException &ex) {
            ROS_ERROR_NAMED("mount", "MountConfigure: %s", ex.what());
        }

        ROS_ERROR_COND_NAMED(!res.success, "mount", "MountCongifure: command plugin service call failed!");

        return res.success;
    }
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MountControlPlugin, mavros::plugin::PluginBase)

/**
 * @brief ManualControls plugin
 * @file manual_controls.cpp
 * @author Matias Nitsche <mnitsche@dc.uba.ar>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Matias Nitsche.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ManualControl.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Manual Control plugin
 */
class ManualControlPlugin : public plugin::PluginBase {
public:
	ManualControlPlugin() : PluginBase(),
		manual_control_nh("~manual_control")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		control_pub = manual_control_nh.advertise<mavros_msgs::ManualControl>("control", 10);
		send_sub = manual_control_nh.subscribe("send", 1, &ManualControlPlugin::send_cb, this);
	}

	Subscriptions get_subscriptions() {
		return {
			make_handler(&ManualControlPlugin::handle_manual_control),
		};
	}

private:
	ros::NodeHandle manual_control_nh;

	ros::Publisher control_pub;
	ros::Subscriber send_sub;

	/* -*- rx handlers -*- */

	void handle_manual_control(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MANUAL_CONTROL &manual_control)
	{
		auto manual_control_msg = boost::make_shared<mavros_msgs::ManualControl>();

		manual_control_msg->header.stamp = ros::Time::now();
		manual_control_msg->x = (manual_control.x / 1000.0);
		manual_control_msg->y = (manual_control.y / 1000.0);
		manual_control_msg->z = (manual_control.z / 1000.0);
		manual_control_msg->r = (manual_control.r / 1000.0);
		manual_control_msg->buttons = manual_control.buttons;

		control_pub.publish(manual_control_msg);
	}

	/* -*- callbacks -*- */

	void send_cb(const mavros_msgs::ManualControl::ConstPtr req)
	{
		mavlink::common::msg::MANUAL_CONTROL msg;
		msg.target = m_uas->get_tgt_system();

		msg.x = req->x;
		msg.y = req->y;
		msg.z = req->z;
		msg.r = req->r;
		msg.buttons = req->buttons;

		UAS_FCU(m_uas)->send_message_ignore_drop(msg);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::ManualControlPlugin, mavros::plugin::PluginBase)

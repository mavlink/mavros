/**
 * @brief Altitude plugin
 * @file altitude.cpp
 * @author Andreas Antener <andreas@uaventure.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Andreas Antener <andreas@uaventure.com>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/Altitude.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Altitude plugin.
 */
class AltitudePlugin : public plugin::PluginBase {
public:
	AltitudePlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		nh.param<std::string>("frame_id", frame_id, "map");
		altitude_pub = nh.advertise<mavros_msgs::Altitude>("altitude", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			make_handler(&AltitudePlugin::handle_altitude),
		};
	}

private:
	ros::NodeHandle nh;
	std::string frame_id;

	ros::Publisher altitude_pub;

	void handle_altitude(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ALTITUDE &altitude)
	{
		auto ros_msg = boost::make_shared<mavros_msgs::Altitude>();
		ros_msg->header = m_uas->synchronized_header(frame_id, altitude.time_usec);

		ros_msg->monotonic = altitude.altitude_monotonic;
		ros_msg->amsl = altitude.altitude_amsl;
		ros_msg->local = altitude.altitude_local;
		ros_msg->relative = altitude.altitude_relative;
		ros_msg->terrain = altitude.altitude_terrain;
		ros_msg->bottom_clearance = altitude.bottom_clearance;

		altitude_pub.publish(ros_msg);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::AltitudePlugin, mavros::plugin::PluginBase)

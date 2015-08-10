/**
 * @brief VFR HUD plugin
 * @file vfr_hud.cpp
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

#include <angles/angles.h>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/VFR_HUD.h>
#include <geometry_msgs/TwistStamped.h>

namespace mavplugin {
/**
 * @brief VFR HUD plugin.
 */
class VfrHudPlugin : public MavRosPlugin {
public:
	VfrHudPlugin() :
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas)
	{
		vfr_pub = nh.advertise<mavros_msgs::VFR_HUD>("vfr_hud", 10);

#ifdef MAVLINK_MSG_ID_WIND
		wind_pub = nh.advertise<geometry_msgs::TwistStamped>("wind_estimation", 10);
#endif
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_VFR_HUD, &VfrHudPlugin::handle_vfr_hud),
#ifdef MAVLINK_MSG_ID_WIND
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_WIND, &VfrHudPlugin::handle_wind),
#endif
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher vfr_pub;
	ros::Publisher wind_pub;

	void handle_vfr_hud(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_vfr_hud_t vfr_hud;
		mavlink_msg_vfr_hud_decode(msg, &vfr_hud);

		auto vmsg = boost::make_shared<mavros_msgs::VFR_HUD>();
		vmsg->header.stamp = ros::Time::now();
		vmsg->airspeed = vfr_hud.airspeed;
		vmsg->groundspeed = vfr_hud.groundspeed;
		vmsg->heading = vfr_hud.heading;
		vmsg->throttle = vfr_hud.throttle / 100.0;	// comes in 0..100 range
		vmsg->altitude = vfr_hud.alt;
		vmsg->climb = vfr_hud.climb;

		vfr_pub.publish(vmsg);
	}

#ifdef MAVLINK_MSG_ID_WIND
	/**
	 * Handle APM specific wind direction estimation message
	 */
	void handle_wind(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_wind_t wind;
		mavlink_msg_wind_decode(msg, &wind);

		const double speed = wind.speed;
		const double course = angles::from_degrees(wind.direction);

		auto twist = boost::make_shared<geometry_msgs::TwistStamped>();
		twist->header.stamp = ros::Time::now();
		// TODO: check math's
		twist->twist.linear.x = speed * std::sin(course);
		twist->twist.linear.y = speed * std::cos(course);
		twist->twist.linear.z = wind.speed_z;

		wind_pub.publish(twist);
	}
#endif
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::VfrHudPlugin, mavplugin::MavRosPlugin)


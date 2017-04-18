/**
 * @brief Automatic dependent surveillance-broadcast Vehicle plugin
 * @file adsb.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ADSBVehicle.h>

namespace mavros {
namespace extra_plugins {
using mavlink::common::ADSB_ALTITUDE_TYPE;
using mavlink::common::ADSB_EMITTER_TYPE;
using mavlink::common::ADSB_FLAGS;

/**
 * @brief ADS-B Vehicle plugin
 *
 * Publish/subscribe Automatic dependent surveillance-broadcast data to/from a vehicle.
 */
class ADSBPlugin : public plugin::PluginBase {
public:
	ADSBPlugin() : PluginBase(),
		adsb_nh("~adsb")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		adsb_nh.param<std::string>("frame_id", frame_id, "adsb_vehicle");

		adsb_pub = adsb_nh.advertise<mavros_msgs::ADSBVehicle>("vehicle", 10);
		adsb_sub = adsb_nh.subscribe("own", 10, &ADSBPlugin::adsb_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&ADSBPlugin::handle_adsb)
		};
	}

private:
	ros::NodeHandle adsb_nh;

	std::string frame_id;

	ros::Publisher adsb_pub;
	ros::Subscriber adsb_sub;

	ADSB_ALTITUDE_TYPE altitude_type;
	ADSB_EMITTER_TYPE emitter_type;

	void handle_adsb(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ADSB_VEHICLE &adsb)
	{
		auto adsb_msg = boost::make_shared<mavros_msgs::ADSBVehicle>();
		const auto &adsb_callsign = adsb.callsign;

		altitude_type = static_cast<ADSB_ALTITUDE_TYPE>(adsb.altitude_type);
		emitter_type = static_cast<ADSB_EMITTER_TYPE>(adsb.emitter_type);

		ros::Time tslc(adsb.tslc, 0);

		adsb_msg->header.stamp = ros::Time::now();	//TODO: request add time_boot_ms to msg definition
		adsb_msg->header.frame_id = frame_id;
		adsb_msg->ICAO_address = adsb.ICAO_address;
		adsb_msg->lla.latitude = adsb.lat / 1E7;
		adsb_msg->lla.longitude = adsb.lon / 1E7;
		adsb_msg->lla.altitude = adsb.altitude;	// in meters, TODO: #693
		adsb_msg->altitude_type = utils::to_string(altitude_type);
		adsb_msg->heading = adsb.heading / 1E2 / 180.0f * M_PI - M_PI;
		adsb_msg->hor_velocity = adsb.hor_velocity / 1E2;
		adsb_msg->ver_velocity = adsb.ver_velocity / 1E2;	// up is positive
		std::copy(adsb_callsign.cbegin(), adsb_callsign.cend(), adsb_msg->callsign.begin());
		adsb_msg->emitter_type = utils::to_string(emitter_type);
		adsb_msg->tslc.data = tslc;
		adsb_msg->data_status = adsb.flags;
		adsb_msg->squawk = adsb.squawk;

		ROS_INFO_STREAM_NAMED("adsb", utils::to_string(adsb.flags));

		adsb_pub.publish(adsb_msg);
	}

	void adsb_cb(const mavros_msgs::ADSBVehicle::ConstPtr &req)
	{
		mavlink::common::msg::ADSB_VEHICLE adsb {};

		adsb.ICAO_address = req->ICAO_address;
		adsb.lat = req->lla.latitude * 1E7;
		adsb.lon = req->lla.longitude * 1E7;
		adsb.altitude = req->lla.altitude;	// in meters, TODO: #693
		adsb.altitude_type = utils::alt_type_from_str(req->altitude_type);
		adsb.heading = req->heading * 1E2 * 180.0f / M_PI + M_PI;
		adsb.hor_velocity = req->hor_velocity * 1E2;
		adsb.ver_velocity = req->ver_velocity * 1E2;	// up is positive
		mavlink::set_string(adsb.callsign, req->callsign);
		adsb.emitter_type = utils::emitter_type_from_str(req->emitter_type);
		adsb.tslc = req->tslc.data.toSec();
		adsb.flags = req->data_status;
		adsb.squawk = req->squawk;

		ROS_INFO_STREAM_NAMED("adsb", utils::to_string(req->data_status));

		UAS_FCU(m_uas)->send_message_ignore_drop(adsb);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ADSBPlugin, mavros::plugin::PluginBase)

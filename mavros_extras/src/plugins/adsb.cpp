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
using mavlink::common::ADSB_EMITTER_TYPE;
using mavlink::common::ADSB_ALTITUDE_TYPE;

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

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		adsb_pub = adsb_nh.advertise<mavros_msgs::ADSBVehicle>("vehicle", 10);
		adsb_sub = adsb_nh.subscribe("send", 10, &ADSBPlugin::adsb_cb, this);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			       make_handler(&ADSBPlugin::handle_adsb)
		};
	}

private:
	ros::NodeHandle adsb_nh;

	ros::Publisher adsb_pub;
	ros::Subscriber adsb_sub;

	void handle_adsb(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ADSB_VEHICLE &adsb)
	{
		auto adsb_msg = boost::make_shared<mavros_msgs::ADSBVehicle>();

		adsb_msg->header.stamp = ros::Time::now();	//TODO: request add time_boot_ms to msg definition
		// [[[cog:
		// def ent(ros, mav=None, scale=None, to_ros=None, to_mav=None):
		//     return (ros, mav or ros, scale, to_ros, to_mav)
		//
		// TR_TAB = (
		// ent('ICAO_address'),
		// ent('callsign', to_ros='mavlink::to_string', to_mav='mavlink::set_string_z({mm}.{mav}, {rmp}->{ros})'),
		// ent('latitude', 'lat', '1e7'),
		// ent('longitude', 'lon', '1e7'),
		// ent('altitude', 'altitude', '1e3'),
		// ent('altitude_type', ),
		// ent('heading', scale='1e2'),
		// ent('hor_velocity', scale='1e2'),
		// ent('ver_velocity', scale='1e2'),
		// ent('altitude_type'),
		// ent('emitter_type'),
		// ent('tslc', to_ros='ros::Duration', to_mav='{mm}.{mav} = {rmp}->{ros}.sec'),
		// ent('flags'),
		// ent('squawk'),
		// )
		//
		// for ros, mav, scale, to_ros, _ in TR_TAB:
		//     if to_ros is None:
		//         scale_ex = '' if scale is None else ' / ' + scale
		//         cog.outl("""adsb_msg->{ros} = adsb.{mav}{scale_ex};""".format(**locals()))
		//     else:
		//         cog.outl("""adsb_msg->{ros} = {to_ros}(adsb.{mav});""".format(**locals()))
		// ]]]
		adsb_msg->ICAO_address = adsb.ICAO_address;
		adsb_msg->callsign = mavlink::to_string(adsb.callsign);
		adsb_msg->latitude = adsb.lat / 1e7;
		adsb_msg->longitude = adsb.lon / 1e7;
		adsb_msg->altitude = adsb.altitude / 1e3;
		adsb_msg->altitude_type = adsb.altitude_type;
		adsb_msg->heading = adsb.heading / 1e2;
		adsb_msg->hor_velocity = adsb.hor_velocity / 1e2;
		adsb_msg->ver_velocity = adsb.ver_velocity / 1e2;
		adsb_msg->altitude_type = adsb.altitude_type;
		adsb_msg->emitter_type = adsb.emitter_type;
		adsb_msg->tslc = ros::Duration(adsb.tslc);
		adsb_msg->flags = adsb.flags;
		adsb_msg->squawk = adsb.squawk;
		// [[[end]]] (checksum: b9c515e7a6fe688b91f4e72e655b9154)

		ROS_DEBUG_STREAM_NAMED("adsb", "ADSB: recv type: " << utils::to_string_enum<ADSB_ALTITUDE_TYPE>(adsb.altitude_type)
				<< " emitter: " << utils::to_string_enum<ADSB_EMITTER_TYPE>(adsb.emitter_type)
				<< " flags: 0x" << std::hex << adsb.flags);

		adsb_pub.publish(adsb_msg);
	}

	void adsb_cb(const mavros_msgs::ADSBVehicle::ConstPtr &req)
	{
		mavlink::common::msg::ADSB_VEHICLE adsb{};

		// [[[cog:
		// for ros, mav, scale, _, to_mav in TR_TAB:
		//     if to_mav is None:
		//         scale_ex = '' if scale is None else ' * ' + scale
		//         cog.outl("""adsb.{mav} = req->{ros}{scale_ex};""".format(**locals()))
		//     else:
		//         cog.outl(to_mav.format(mm='adsb', rmp='req', **locals()) + ';')
		// ]]]
		adsb.ICAO_address = req->ICAO_address;
		mavlink::set_string_z(adsb.callsign, req->callsign);
		adsb.lat = req->latitude * 1e7;
		adsb.lon = req->longitude * 1e7;
		adsb.altitude = req->altitude * 1e3;
		adsb.altitude_type = req->altitude_type;
		adsb.heading = req->heading * 1e2;
		adsb.hor_velocity = req->hor_velocity * 1e2;
		adsb.ver_velocity = req->ver_velocity * 1e2;
		adsb.altitude_type = req->altitude_type;
		adsb.emitter_type = req->emitter_type;
		adsb.tslc = req->tslc.sec;
		adsb.flags = req->flags;
		adsb.squawk = req->squawk;
		// [[[end]]] (checksum: 8583d9ea3a3eefae10ccd7037c06b46d)

		ROS_DEBUG_STREAM_NAMED("adsb", "ADSB: send type: " << utils::to_string_enum<ADSB_ALTITUDE_TYPE>(adsb.altitude_type)
				<< " emitter: " << utils::to_string_enum<ADSB_EMITTER_TYPE>(adsb.emitter_type)
				<< " flags: 0x" << std::hex << adsb.flags);

		UAS_FCU(m_uas)->send_message_ignore_drop(adsb);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ADSBPlugin, mavros::plugin::PluginBase)

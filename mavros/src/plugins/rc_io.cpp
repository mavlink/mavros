/**
 * @brief RC IO plugin
 * @file rc_io.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros/RCIn.h>
#include <mavros/RCOut.h>
#include <mavros/OverrideRCIn.h>

namespace mavplugin {
/**
 * @brief RC IO plugin
 */
class RCIOPlugin : public MavRosPlugin {
public:
	RCIOPlugin() :
		rc_nh("~rc"),
		uas(nullptr),
		raw_rc_in(0),
		raw_rc_out(0),
		has_rc_channels_msg(false)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		rc_in_pub = rc_nh.advertise<mavros::RCIn>("in", 10);
		rc_out_pub = rc_nh.advertise<mavros::RCOut>("out", 10);
		override_sub = rc_nh.subscribe("override", 10, &RCIOPlugin::override_cb, this);

		uas->sig_connection_changed.connect(boost::bind(&RCIOPlugin::connection_cb, this, _1));
	};

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_RC_CHANNELS_RAW, &RCIOPlugin::handle_rc_channels_raw),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_RC_CHANNELS, &RCIOPlugin::handle_rc_channels),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, &RCIOPlugin::handle_servo_output_raw),
		};
	}

private:
	std::recursive_mutex mutex;
	ros::NodeHandle rc_nh;
	UAS *uas;

	std::vector<uint16_t> raw_rc_in;
	std::vector<uint16_t> raw_rc_out;
	bool has_rc_channels_msg;

	ros::Publisher rc_in_pub;
	ros::Publisher rc_out_pub;
	ros::Subscriber override_sub;

	/* -*- rx handlers -*- */

	void handle_rc_channels_raw(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_rc_channels_raw_t port;
		mavlink_msg_rc_channels_raw_decode(msg, &port);
		lock_guard lock(mutex);

		/* if we receive RC_CHANNELS, drop RC_CHANNELS_RAW */
		if (has_rc_channels_msg)
			return;

		size_t offset = port.port * 8;
		if (raw_rc_in.size() < offset + 8)
			raw_rc_in.resize(offset + 8);

#define SET_RC_IN(mavidx)	\
	raw_rc_in[offset + mavidx - 1] = port.chan ## mavidx ## _raw
		SET_RC_IN(1);
		SET_RC_IN(2);
		SET_RC_IN(3);
		SET_RC_IN(4);
		SET_RC_IN(5);
		SET_RC_IN(6);
		SET_RC_IN(7);
		SET_RC_IN(8);
#undef SET_RC_IN

		auto rcin_msg = boost::make_shared<mavros::RCIn>();

		rcin_msg->header.stamp = uas->synchronise_stamp(port.time_boot_ms);
		rcin_msg->rssi = port.rssi;
		rcin_msg->channels = raw_rc_in;

		rc_in_pub.publish(rcin_msg);
	}

	void handle_rc_channels(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_rc_channels_t channels;
		mavlink_msg_rc_channels_decode(msg, &channels);
		lock_guard lock(mutex);

		ROS_INFO_COND_NAMED(!has_rc_channels_msg, "rc", "RC_CHANNELS message detected!");
		has_rc_channels_msg = true;

		if (channels.chancount > 18) {
			raw_rc_in.resize(18);
			ROS_WARN_THROTTLE_NAMED(60, "rc",
					"FCU receives %u RC channels, but RC_CHANNELS can store 18",
					channels.chancount);
		}
		else
			raw_rc_in.resize(channels.chancount);

#define IFSET_RC_IN(mavidx)				\
	if (channels.chancount >= mavidx)	\
		raw_rc_in[mavidx - 1] = channels.chan ## mavidx ## _raw
		IFSET_RC_IN(1);
		IFSET_RC_IN(2);
		IFSET_RC_IN(3);
		IFSET_RC_IN(4);
		IFSET_RC_IN(5);
		IFSET_RC_IN(6);
		IFSET_RC_IN(7);
		IFSET_RC_IN(8);
		IFSET_RC_IN(9);
		IFSET_RC_IN(10);
		IFSET_RC_IN(11);
		IFSET_RC_IN(12);
		IFSET_RC_IN(13);
		IFSET_RC_IN(14);
		IFSET_RC_IN(15);
		IFSET_RC_IN(16);
		IFSET_RC_IN(17);
		IFSET_RC_IN(18);
#undef IFSET_RC_IN

		auto rcin_msg = boost::make_shared<mavros::RCIn>();

		rcin_msg->header.stamp = uas->synchronise_stamp(channels.time_boot_ms);
		rcin_msg->rssi = channels.rssi;
		rcin_msg->channels = raw_rc_in;

		rc_in_pub.publish(rcin_msg);
	}

	void handle_servo_output_raw(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_servo_output_raw_t port;
		mavlink_msg_servo_output_raw_decode(msg, &port);
		lock_guard lock(mutex);

		size_t offset = port.port * 8;
		if (raw_rc_out.size() < offset + 8)
			raw_rc_out.resize(offset + 8);

#define SET_RC_OUT(mavidx)	\
	raw_rc_out[offset + mavidx - 1] = port.servo ## mavidx ## _raw
		SET_RC_OUT(1);
		SET_RC_OUT(2);
		SET_RC_OUT(3);
		SET_RC_OUT(4);
		SET_RC_OUT(5);
		SET_RC_OUT(6);
		SET_RC_OUT(7);
		SET_RC_OUT(8);
#undef SET_RC_OUT

		auto rcout_msg = boost::make_shared<mavros::RCOut>();

		// XXX: Why time_usec id 32 bit? We should test that.
		uint64_t time_usec = port.time_usec;

		rcout_msg->header.stamp = uas->synchronise_stamp(time_usec);
		rcout_msg->channels = raw_rc_out;

		rc_out_pub.publish(rcout_msg);
	}

	/* -*- low-level send functions -*- */

	void rc_channels_override(const boost::array<uint16_t, 8> &channels) {
		mavlink_message_t msg;

		mavlink_msg_rc_channels_override_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas),
				channels[0],
				channels[1],
				channels[2],
				channels[3],
				channels[4],
				channels[5],
				channels[6],
				channels[7]
				);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */

	void connection_cb(bool connected) {
		lock_guard lock(mutex);
		raw_rc_in.clear();
		raw_rc_out.clear();
		has_rc_channels_msg = false;
	}

	void override_cb(const mavros::OverrideRCIn::ConstPtr req) {
		if (!uas->is_ardupilotmega())
			ROS_WARN_THROTTLE_NAMED(30, "rc", "RC override not supported by this FCU!");

		rc_channels_override(req->channels);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::RCIOPlugin, mavplugin::MavRosPlugin)


/**
 * @brief RC IO plugin
 * @file rc_io.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/OverrideRCIn.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief RC IO plugin
 */
class RCIOPlugin : public plugin::PluginBase {
public:
	RCIOPlugin() : PluginBase(),
		rc_nh("~rc"),
		raw_rc_in(0),
		raw_rc_out(0),
		has_rc_channels_msg(false)
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		rc_in_pub = rc_nh.advertise<mavros_msgs::RCIn>("in", 10);
		rc_out_pub = rc_nh.advertise<mavros_msgs::RCOut>("out", 10);
		override_sub = rc_nh.subscribe("override", 10, &RCIOPlugin::override_cb, this);

		enable_connection_cb();
	};

	Subscriptions get_subscriptions() override {
		return {
			       make_handler(&RCIOPlugin::handle_rc_channels_raw),
			       make_handler(&RCIOPlugin::handle_rc_channels),
			       make_handler(&RCIOPlugin::handle_servo_output_raw),
		};
	}

private:
	using lock_guard = std::lock_guard<std::mutex>;
	std::mutex mutex;
	ros::NodeHandle rc_nh;

	std::vector<uint16_t> raw_rc_in;
	std::vector<uint16_t> raw_rc_out;
	std::atomic<bool> has_rc_channels_msg;

	ros::Publisher rc_in_pub;
	ros::Publisher rc_out_pub;
	ros::Subscriber override_sub;

	/* -*- rx handlers -*- */

	void handle_rc_channels_raw(const mavlink::mavlink_message_t *msg, mavlink::common::msg::RC_CHANNELS_RAW &port)
	{
		/* if we receive RC_CHANNELS, drop RC_CHANNELS_RAW */
		if (has_rc_channels_msg)
			return;

		lock_guard lock(mutex);

		size_t offset = port.port * 8;
		if (raw_rc_in.size() < offset + 8)
			raw_rc_in.resize(offset + 8);

		// [[[cog:
		// import cog
		// for i in range(1, 9):
		//     cog.outl("raw_rc_in[offset + %d] = port.chan%d_raw;" % (i - 1, i))
		// ]]]
		raw_rc_in[offset + 0] = port.chan1_raw;
		raw_rc_in[offset + 1] = port.chan2_raw;
		raw_rc_in[offset + 2] = port.chan3_raw;
		raw_rc_in[offset + 3] = port.chan4_raw;
		raw_rc_in[offset + 4] = port.chan5_raw;
		raw_rc_in[offset + 5] = port.chan6_raw;
		raw_rc_in[offset + 6] = port.chan7_raw;
		raw_rc_in[offset + 7] = port.chan8_raw;
		// [[[end]]] (checksum: fcb14b1ddfff9ce7dd02f5bd03825cff)

		auto rcin_msg = boost::make_shared<mavros_msgs::RCIn>();

		rcin_msg->header.stamp = m_uas->synchronise_stamp(port.time_boot_ms);
		rcin_msg->rssi = port.rssi;
		rcin_msg->channels = raw_rc_in;

		rc_in_pub.publish(rcin_msg);
	}

	void handle_rc_channels(const mavlink::mavlink_message_t *msg, mavlink::common::msg::RC_CHANNELS &channels)
	{
		constexpr size_t MAX_CHANCNT = 18;
		lock_guard lock(mutex);

		ROS_INFO_COND_NAMED(!has_rc_channels_msg, "rc", "RC_CHANNELS message detected!");
		has_rc_channels_msg = true;

		if (channels.chancount > MAX_CHANCNT) {
			ROS_WARN_THROTTLE_NAMED(60, "rc",
						"FCU receives %u RC channels, but RC_CHANNELS can store %zu",
						channels.chancount, MAX_CHANCNT);

			channels.chancount = MAX_CHANCNT;
		}

		raw_rc_in.resize(channels.chancount);

		// switch works as start point selector.
		switch (channels.chancount) {
		// [[[cog:
		// for i in range(18, 0, -1):
		//     cog.outl("case %2d: raw_rc_in[%2d] = channels.chan%d_raw;" % (i, i - 1, i))
		// ]]]
		case 18: raw_rc_in[17] = channels.chan18_raw;
		case 17: raw_rc_in[16] = channels.chan17_raw;
		case 16: raw_rc_in[15] = channels.chan16_raw;
		case 15: raw_rc_in[14] = channels.chan15_raw;
		case 14: raw_rc_in[13] = channels.chan14_raw;
		case 13: raw_rc_in[12] = channels.chan13_raw;
		case 12: raw_rc_in[11] = channels.chan12_raw;
		case 11: raw_rc_in[10] = channels.chan11_raw;
		case 10: raw_rc_in[ 9] = channels.chan10_raw;
		case  9: raw_rc_in[ 8] = channels.chan9_raw;
		case  8: raw_rc_in[ 7] = channels.chan8_raw;
		case  7: raw_rc_in[ 6] = channels.chan7_raw;
		case  6: raw_rc_in[ 5] = channels.chan6_raw;
		case  5: raw_rc_in[ 4] = channels.chan5_raw;
		case  4: raw_rc_in[ 3] = channels.chan4_raw;
		case  3: raw_rc_in[ 2] = channels.chan3_raw;
		case  2: raw_rc_in[ 1] = channels.chan2_raw;
		case  1: raw_rc_in[ 0] = channels.chan1_raw;
		// [[[end]]] (checksum: 56e9ab5407bd2c864abde230a6cf3fed)
		case  0: break;
		}

		auto rcin_msg = boost::make_shared<mavros_msgs::RCIn>();

		rcin_msg->header.stamp = m_uas->synchronise_stamp(channels.time_boot_ms);
		rcin_msg->rssi = channels.rssi;
		rcin_msg->channels = raw_rc_in;

		rc_in_pub.publish(rcin_msg);
	}

	void handle_servo_output_raw(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SERVO_OUTPUT_RAW &port)
	{
		lock_guard lock(mutex);

		uint8_t num_channels;

		// If using Mavlink protocol v2, number of available servo channels is 16;
		// otherwise, 8
		if (msg->magic == MAVLINK_STX)
			num_channels = 16;
		else
			num_channels = 8;

		size_t offset = port.port * num_channels;
		if (raw_rc_out.size() < offset + num_channels)
			raw_rc_out.resize(offset + num_channels);

		// [[[cog:
		// for i in range(1, 9):
		//     cog.outl("raw_rc_out[offset + %d] = port.servo%d_raw;" % (i - 1, i))
		// ]]]
		raw_rc_out[offset + 0] = port.servo1_raw;
		raw_rc_out[offset + 1] = port.servo2_raw;
		raw_rc_out[offset + 2] = port.servo3_raw;
		raw_rc_out[offset + 3] = port.servo4_raw;
		raw_rc_out[offset + 4] = port.servo5_raw;
		raw_rc_out[offset + 5] = port.servo6_raw;
		raw_rc_out[offset + 6] = port.servo7_raw;
		raw_rc_out[offset + 7] = port.servo8_raw;
		// [[[end]]] (checksum: 946d524fe9fbaa3e52fbdf8a905fbf0f)
		if (msg->magic == MAVLINK_STX) {
			// [[[cog:
			// for i in range(9, 17):
			//     cog.outl("raw_rc_out[offset + %d] = port.servo%d_raw;" % (i - 1, i))
			// ]]]
			raw_rc_out[offset + 8] = port.servo9_raw;
			raw_rc_out[offset + 9] = port.servo10_raw;
			raw_rc_out[offset + 10] = port.servo11_raw;
			raw_rc_out[offset + 11] = port.servo12_raw;
			raw_rc_out[offset + 12] = port.servo13_raw;
			raw_rc_out[offset + 13] = port.servo14_raw;
			raw_rc_out[offset + 14] = port.servo15_raw;
			raw_rc_out[offset + 15] = port.servo16_raw;
			// [[[end]]] (checksum: 60a386cba6faa126ee7dfe1b22f50398)
		}

		auto rcout_msg = boost::make_shared<mavros_msgs::RCOut>();

		// XXX: Why time_usec is 32 bit? We should test that.
		uint64_t time_usec = port.time_usec;

		rcout_msg->header.stamp = m_uas->synchronise_stamp(time_usec);
		rcout_msg->channels = raw_rc_out;

		rc_out_pub.publish(rcout_msg);
	}

	/* -*- callbacks -*- */

	void connection_cb(bool connected) override
	{
		lock_guard lock(mutex);
		raw_rc_in.clear();
		raw_rc_out.clear();
		has_rc_channels_msg = false;
	}

	void override_cb(const mavros_msgs::OverrideRCIn::ConstPtr req)
	{
		if (!m_uas->is_ardupilotmega() && !m_uas->is_px4())
			ROS_WARN_THROTTLE_NAMED(30, "rc", "RC override not supported by this FCU!");

		mavlink::common::msg::RC_CHANNELS_OVERRIDE ovr = {};
		ovr.target_system = m_uas->get_tgt_system();
		ovr.target_component = m_uas->get_tgt_component();

		// [[[cog:
		// for i in range(1, 9):
		//     cog.outl("ovr.chan%d_raw = req->channels[%d];" % (i, i - 1))
		// ]]]
		ovr.chan1_raw = req->channels[0];
		ovr.chan2_raw = req->channels[1];
		ovr.chan3_raw = req->channels[2];
		ovr.chan4_raw = req->channels[3];
		ovr.chan5_raw = req->channels[4];
		ovr.chan6_raw = req->channels[5];
		ovr.chan7_raw = req->channels[6];
		ovr.chan8_raw = req->channels[7];
		// [[[end]]] (checksum: bd27f3e85f5ab614ce1332ae3f4c6ebd)

		UAS_FCU(m_uas)->send_message_ignore_drop(ovr);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::RCIOPlugin, mavros::plugin::PluginBase)

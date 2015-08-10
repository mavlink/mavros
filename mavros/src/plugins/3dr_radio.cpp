/**
 * @brief 3DR Radio status plugin
 * @file 3dr_radio.cpp
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

#include <mavros_msgs/RadioStatus.h>

namespace mavplugin {
/**
 * @brief 3DR Radio plugin.
 */
class TDRRadioPlugin : public MavRosPlugin {
public:
	TDRRadioPlugin() :
		nh("~"),
		uas(nullptr),
		has_radio_status(false),
		diag_added(false),
		low_rssi(0)
	{ }

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		nh.param("tdr_radio/low_rssi", low_rssi, 40);

		status_pub = nh.advertise<mavros_msgs::RadioStatus>("radio_status", 10);

		uas->sig_connection_changed.connect(boost::bind(&TDRRadioPlugin::connection_cb, this, _1));
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_RADIO_STATUS, &TDRRadioPlugin::handle_radio_status),
#ifdef MAVLINK_MSG_ID_RADIO
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_RADIO, &TDRRadioPlugin::handle_radio),
#endif
		};
	}

private:
	ros::NodeHandle nh;
	UAS *uas;

	bool has_radio_status;
	bool diag_added;
	int low_rssi;

	ros::Publisher status_pub;

	std::recursive_mutex diag_mutex;
	mavros_msgs::RadioStatus::Ptr last_status;

	/* -*- message handlers -*- */

	void handle_radio_status(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_radio_status_t rst;
		mavlink_msg_radio_status_decode(msg, &rst);
		has_radio_status = true;
		handle_message(rst, sysid, compid);
	}

#ifdef MAVLINK_MSG_ID_RADIO
	void handle_radio(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (has_radio_status)
			return;

		// actually the same data, but from earlier modems
		mavlink_radio_t rst;
		mavlink_msg_radio_decode(msg, &rst);
		handle_message(rst, sysid, compid);
	}
#endif

	template<typename msgT>
	void handle_message(msgT &rst, uint8_t sysid, uint8_t compid) {
		if (sysid != '3' || compid != 'D')
			ROS_WARN_THROTTLE_NAMED(30, "radio", "RADIO_STATUS not from 3DR modem?");

		auto msg = boost::make_shared<mavros_msgs::RadioStatus>();

		msg->header.stamp = ros::Time::now();

#define RST_COPY(field)	msg->field = rst.field
		RST_COPY(rssi);
		RST_COPY(remrssi);
		RST_COPY(txbuf);
		RST_COPY(noise);
		RST_COPY(remnoise);
		RST_COPY(rxerrors);
		RST_COPY(fixed);
#undef RST_COPY

		// valid for 3DR modem
		msg->rssi_dbm = (rst.rssi / 1.9) - 127;
		msg->remrssi_dbm = (rst.remrssi / 1.9) - 127;

		// add diag at first event
		if (!diag_added) {
			UAS_DIAG(uas).add("3DR Radio", this, &TDRRadioPlugin::diag_run);
			diag_added = true;
		}

		// store last status for diag
		{
			lock_guard lock(diag_mutex);
			last_status = msg;
		}

		status_pub.publish(msg);
	}


	void diag_run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(diag_mutex);

		if (!last_status) {
			stat.summary(2, "No data");
			return;
		}
		else if (last_status->rssi < low_rssi)
			stat.summary(1, "Low RSSI");
		else if (last_status->remrssi < low_rssi)
			stat.summary(1, "Low remote RSSI");
		else
			stat.summary(0, "Normal");

		stat.addf("RSSI", "%u", last_status->rssi);
		stat.addf("RSSI (dBm)", "%.1f", last_status->rssi_dbm);
		stat.addf("Remote RSSI", "%u", last_status->remrssi);
		stat.addf("Remote RSSI (dBm)", "%.1f", last_status->remrssi_dbm);
		stat.addf("Tx buffer (%)", "%u", last_status->txbuf);
		stat.addf("Noice level", "%u", last_status->noise);
		stat.addf("Remote noice level", "%u", last_status->remnoise);
		stat.addf("Rx errors", "%u", last_status->rxerrors);
		stat.addf("Fixed", "%u", last_status->fixed);
	}

	void connection_cb(bool connected) {
		UAS_DIAG(uas).removeByName("3DR Radio");
		diag_added = false;
	}

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::TDRRadioPlugin, mavplugin::MavRosPlugin)


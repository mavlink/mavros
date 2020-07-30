/**
 * @brief 3DR Radio status plugin
 * @file 3dr_radio.cpp
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

#include <mavros_msgs/RadioStatus.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief 3DR Radio plugin.
 */
class TDRRadioPlugin : public plugin::PluginBase {
public:
	TDRRadioPlugin() : PluginBase(),
		nh("~"),
		has_radio_status(false),
		diag_added(false),
		low_rssi(0)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		nh.param("tdr_radio/low_rssi", low_rssi, 40);

		status_pub = nh.advertise<mavros_msgs::RadioStatus>("radio_status", 10);

		enable_connection_cb();
	}

	Subscriptions get_subscriptions()
	{
		return {
			make_handler(&TDRRadioPlugin::handle_radio_status),
			make_handler(&TDRRadioPlugin::handle_radio),
		};
	}

private:
	ros::NodeHandle nh;

	bool has_radio_status;
	bool diag_added;
	int low_rssi;

	ros::Publisher status_pub;

	std::mutex diag_mutex;
	mavros_msgs::RadioStatus::Ptr last_status;

	/* -*- message handlers -*- */

	void handle_radio_status(const mavlink::mavlink_message_t *msg, mavlink::common::msg::RADIO_STATUS &rst)
	{
		has_radio_status = true;
		handle_message(msg, rst);
	}

	void handle_radio(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::RADIO &rst)
	{
		if (has_radio_status)
			return;

		// actually the same data, but from earlier modems
		handle_message(msg, rst);
	}

	template<typename msgT>
	void handle_message(const mavlink::mavlink_message_t *mmsg, msgT &rst)
	{
		if (mmsg->sysid != '3' || mmsg->compid != 'D')
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
			UAS_DIAG(m_uas).add("3DR Radio", this, &TDRRadioPlugin::diag_run);
			diag_added = true;
		}

		// store last status for diag
		{
			std::lock_guard<std::mutex> lock(diag_mutex);
			last_status = msg;
		}

		status_pub.publish(msg);
	}


	void diag_run(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		std::lock_guard<std::mutex> lock(diag_mutex);

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

	void connection_cb(bool connected) override
	{
		UAS_DIAG(m_uas).removeByName("3DR Radio");
		diag_added = false;
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::TDRRadioPlugin, mavros::plugin::PluginBase)

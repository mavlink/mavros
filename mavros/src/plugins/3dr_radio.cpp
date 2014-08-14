/**
 * @brief 3DR Radio status plugin
 * @file 3dr_radio.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros/RadioStatus.h>

namespace mavplugin {

class TDRRadioStatus : public diagnostic_updater::DiagnosticTask
{
public:
	TDRRadioStatus(const std::string name, uint8_t _low_rssi) :
		diagnostic_updater::DiagnosticTask(name),
		data_received(false),
		low_rssi(_low_rssi),
		last_rst{}
	{ }


	template <typename msgT>
	void set(msgT &rst) {
		lock_guard lock(mutex);
		data_received = true;
#define RST_COPY(field)	last_rst.field = rst.field
		RST_COPY(rssi);
		RST_COPY(remrssi);
		RST_COPY(txbuf);
		RST_COPY(noise);
		RST_COPY(remnoise);
		RST_COPY(rxerrors);
		RST_COPY(fixed);
#undef RST_COPY
	}

	/**
	 * @todo check RSSI warning level
	 */
	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);

		if (!data_received)
			stat.summary(2, "No data");
		else if (last_rst.rssi < low_rssi)
			stat.summary(1, "Low RSSI");
		else if (last_rst.remrssi < low_rssi)
			stat.summary(1, "Low remote RSSI");
		else
			stat.summary(0, "Normal");

		float rssi_dbm = (last_rst.rssi / 1.9) - 127;
		float remrssi_dbm = (last_rst.remrssi / 1.9) - 127;

		stat.addf("RSSI", "%u", last_rst.rssi);
		stat.addf("RSSI (dBm)", "%.1f", rssi_dbm);
		stat.addf("Remote RSSI", "%u", last_rst.remrssi);
		stat.addf("Remote RSSI (dBm)", "%.1f", remrssi_dbm);
		stat.addf("Tx buffer (%)", "%u", last_rst.txbuf);
		stat.addf("Noice level", "%u", last_rst.noise);
		stat.addf("Remote noice level", "%u", last_rst.remnoise);
		stat.addf("Rx errors", "%u", last_rst.rxerrors);
		stat.addf("Fixed", "%u", last_rst.fixed);
	}

private:
	std::recursive_mutex mutex;
	mavlink_radio_status_t last_rst;
	bool data_received;
	const uint8_t low_rssi;
};


/**
 * @brief 3DR Radio plugin.
 */
class TDRRadioPlugin : public MavRosPlugin {
public:
	TDRRadioPlugin() :
		tdr_diag("3DR Radio", 40),
		has_radio_status(false)
	{ }

	void initialize(UAS &uas,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		diag_updater.add(tdr_diag);
		status_pub = nh.advertise<mavros::RadioStatus>("radio_status", 10);
	}

	std::string const get_name() const {
		return "3DRRadio";
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
	TDRRadioStatus tdr_diag;
	bool has_radio_status;

	ros::Publisher status_pub;

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

		tdr_diag.set(rst);

		mavros::RadioStatusPtr msg = boost::make_shared<mavros::RadioStatus>();

#define RST_COPY(field)	msg->field = rst.field
		RST_COPY(rssi);
		RST_COPY(remrssi);
		RST_COPY(txbuf);
		RST_COPY(noise);
		RST_COPY(remnoise);
		RST_COPY(rxerrors);
		RST_COPY(fixed);
#undef RST_COPY

		msg->header.stamp = ros::Time::now();
		status_pub.publish(msg);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::TDRRadioPlugin, mavplugin::MavRosPlugin)


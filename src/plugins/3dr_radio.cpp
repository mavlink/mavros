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

namespace mavplugin {

class TDRRadioStatus : public diagnostic_updater::DiagnosticTask
{
public:
	TDRRadioStatus(const std::string name) :
		diagnostic_updater::DiagnosticTask(name),
		data_received(false)
	{
		memset(&last_rst, 0, sizeof(last_rst));
	}


	void set(mavlink_radio_status_t &rst) {
		boost::recursive_mutex::scoped_lock lock(mutex);
		data_received = true;
		last_rst = rst;
	}

	void set(mavlink_radio_t &rst) {
		boost::recursive_mutex::scoped_lock lock(mutex);
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
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (!data_received)
			stat.summary(2, "No data");
		else if (last_rst.rssi < 20)
			stat.summary(1, "Low RSSI");
		else if (last_rst.remrssi < 20)
			stat.summary(1, "Low remote RSSI");
		else
			stat.summary(0, "Normal");

		stat.addf("RSSI", "%u", last_rst.rssi);
		stat.addf("Remote RSSI", "%u", last_rst.remrssi);
		stat.addf("Tx buffer (%)", "%u", last_rst.txbuf);
		stat.addf("Noice level", "%u", last_rst.noise);
		stat.addf("Remote noice level", "%u", last_rst.remnoise);
		stat.addf("Rx errors", "%u", last_rst.rxerrors);
		stat.addf("Fixed", "%u", last_rst.fixed);
	}

private:
	boost::recursive_mutex mutex;
	mavlink_radio_status_t last_rst;
	bool data_received;
};


/**
 * @brief 3DR Radio plugin.
 */
class TDRRadioPlugin : public MavRosPlugin {
public:
	TDRRadioPlugin() :
		tdr_diag("3DR Radio")
	{ }

	void initialize(UAS &uas,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		diag_updater.add(tdr_diag);
	}

	std::string const get_name() const {
		return "3DRRadio";
	}

	std::vector<uint8_t> const get_supported_messages() const {
		return {
			MAVLINK_MSG_ID_RADIO_STATUS
#ifdef MAVLINK_MSG_ID_RADIO
			, MAVLINK_MSG_ID_RADIO
#endif
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_RADIO_STATUS:
			{
				mavlink_radio_status_t rst;
				mavlink_msg_radio_status_decode(msg, &rst);
				tdr_diag.set(rst);
			}
			break;

#ifdef MAVLINK_MSG_ID_RADIO
		case MAVLINK_MSG_ID_RADIO:
			{
				// actually the same data, but from earlier modems
				mavlink_radio_t rst;
				mavlink_msg_radio_decode(msg, &rst);
				tdr_diag.set(rst);
			}
			break;
#endif
		}
	}

private:
	TDRRadioStatus tdr_diag;
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::TDRRadioPlugin, mavplugin::MavRosPlugin)


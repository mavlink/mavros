/**
 * @brief System Status plugin
 * @file sys_status.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013 Vladimir Ermakov.
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
#include <boost/date_time/posix_time/posix_time.hpp>

#include <mavros/State.h>
#include <mavros/BatteryStatus.h>
#include <mavros/StreamRate.h>

namespace mavplugin {

/**
 * Heartbeat status publisher
 *
 * Based on diagnistic_updater::FrequencyStatus
 */
class HeartbeatStatus : public diagnostic_updater::DiagnosticTask
{
public:
	HeartbeatStatus(const std::string name, size_t win_size) :
		diagnostic_updater::DiagnosticTask(name),
		window_size_(win_size),
		min_freq_(0.2),
		max_freq_(100),
		tolerance_(0.1),
		times_(win_size),
		seq_nums_(win_size)
	{
		clear();
	}

	void clear() {
		boost::recursive_mutex::scoped_lock lock(mutex);
		ros::Time curtime = ros::Time::now();
		count_ = 0;

		for (int i = 0; i < window_size_; i++)
		{
			times_[i] = curtime;
			seq_nums_[i] = count_;
		}

		hist_indx_ = 0;
	}

	void tick(mavlink_heartbeat_t &hb_struct) {
		boost::recursive_mutex::scoped_lock lock(mutex);
		count_++;
		last_hb = hb_struct;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		boost::recursive_mutex::scoped_lock lock(mutex);
		ros::Time curtime = ros::Time::now();
		int curseq = count_;
		int events = curseq - seq_nums_[hist_indx_];
		double window = (curtime - times_[hist_indx_]).toSec();
		double freq = events / window;
		seq_nums_[hist_indx_] = curseq;
		times_[hist_indx_] = curtime;
		hist_indx_ = (hist_indx_ + 1) % window_size_;

		if (events == 0) {
			stat.summary(2, "No events recorded.");
		}
		else if (freq < min_freq_ * (1 - tolerance_)) {
			stat.summary(1, "Frequency too low.");
		}
		else if (freq > max_freq_ * (1 + tolerance_)) {
			stat.summary(1, "Frequency too high.");
		}
		else {
			stat.summary(0, "Normal");
		}

		stat.addf("Events in window", "%d", events);
		stat.addf("Events since startup", "%d", count_);
		stat.addf("Duration of window (s)", "%f", window);
		stat.addf("Actual frequency (Hz)", "%f", freq);
		stat.addf("MAV Type", "%u", last_hb.type);
		stat.addf("Autopilot type", "%u", last_hb.autopilot);
		stat.addf("Autopilot base mode", "0x%02X", last_hb.base_mode);
		stat.addf("Autopilot custom mode", "0x%08X", last_hb.custom_mode);
		stat.addf("Autopilot system status", "%u", last_hb.system_status);
	}

private:
	int count_;
	std::vector<ros::Time> times_;
	std::vector<int> seq_nums_;
	int hist_indx_;
	boost::recursive_mutex mutex;
	const size_t window_size_;
	const double min_freq_;
	const double max_freq_;
	const double tolerance_;
	mavlink_heartbeat_t last_hb;
};


class SystemStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
	SystemStatusDiag(const std::string name) :
		diagnostic_updater::DiagnosticTask(name)
	{};

	void set(mavlink_sys_status_t &st) {
		boost::recursive_mutex::scoped_lock lock(mutex);
		last_st = st;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if ((last_st.onboard_control_sensors_health & last_st.onboard_control_sensors_enabled)
				!= last_st.onboard_control_sensors_enabled)
			stat.summary(2, "Sensor helth");
		else
			stat.summary(0, "Normal");

		stat.addf("Sensor present", "0x%08X", last_st.onboard_control_sensors_present);
		stat.addf("Sensor enabled", "0x%08X", last_st.onboard_control_sensors_enabled);
		stat.addf("Sensor helth", "0x%08X", last_st.onboard_control_sensors_health);
		stat.addf("CPU Load (%)", "%.1f", last_st.load / 10.0);
		stat.addf("Drop rate (%)", "%.1f", last_st.drop_rate_comm / 10.0);
		stat.addf("Errors comm", "%d", last_st.errors_comm);
		stat.addf("Errors count #1", "%d", last_st.errors_count1);
		stat.addf("Errors count #2", "%d", last_st.errors_count2);
		stat.addf("Errors count #3", "%d", last_st.errors_count3);
		stat.addf("Errors count #4", "%d", last_st.errors_count4);
	}

private:
	boost::recursive_mutex mutex;
	mavlink_sys_status_t last_st;
};


class BatteryStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
	BatteryStatusDiag(const std::string name) :
		diagnostic_updater::DiagnosticTask(name),
		voltage(-1.0),
		current(0.0),
		remaining(0.0),
		min_voltage(6)
	{};

	void set_min_voltage(float volt) {
		boost::recursive_mutex::scoped_lock lock(mutex);
		min_voltage = volt;
	}

	void set(float volt, float curr, float rem) {
		boost::recursive_mutex::scoped_lock lock(mutex);
		voltage = volt;
		current = curr;
		remaining = rem;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (voltage < 0)
			stat.summary(2, "No data");
		else if (voltage < min_voltage)
			stat.summary(1, "Low voltage");
		else
			stat.summary(0, "Normal");

		stat.addf("Voltage", "%.2f", voltage);
		stat.addf("Current", "%.1f", current);
		stat.addf("Remaining", "%.1f", remaining * 100);
	}

private:
	boost::recursive_mutex mutex;
	float voltage;
	float current;
	float remaining;
	float min_voltage;
};


class MemInfo : public diagnostic_updater::DiagnosticTask
{
public:
	MemInfo(const std::string name) :
		diagnostic_updater::DiagnosticTask(name),
		freemem(-1),
		brkval(0)
	{};

	void set(uint16_t f, uint16_t b) {
		boost::recursive_mutex::scoped_lock lock(mutex);
		freemem = f;
		brkval = b;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (freemem < 0)
			stat.summary(2, "No data");
		else if (freemem < 200)
			stat.summary(1, "Low mem");
		else
			stat.summary(0, "Normal");

		stat.addf("Free memory (B)", "%zd", freemem);
		stat.addf("Heap top", "0x%04X", brkval);
	}

private:
	boost::recursive_mutex mutex;
	ssize_t freemem;
	uint16_t brkval;
};


class HwStatus : public diagnostic_updater::DiagnosticTask
{
public:
	HwStatus(const std::string name) :
		diagnostic_updater::DiagnosticTask(name),
		vcc(-1.0),
		i2cerr(0),
		i2cerr_last(0)
	{};

	void set(uint16_t v, uint8_t e) {
		boost::recursive_mutex::scoped_lock lock(mutex);
		vcc = v / 1000.0;
		i2cerr = e;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (vcc < 0)
			stat.summary(2, "No data");
		else if (vcc < 4.5)
			stat.summary(1, "Low voltage");
		else if (i2cerr != i2cerr_last) {
			i2cerr_last = i2cerr;
			stat.summary(1, "New I2C error");
		}
		else
			stat.summary(0, "Normal");

		stat.addf("Core voltage", "%f", vcc);
		stat.addf("I2C errors", "%zu", i2cerr);
	}

private:
	boost::recursive_mutex mutex;
	float vcc;
	size_t i2cerr;
	size_t i2cerr_last;
};


/**
 * @brief System status plugin.
 * Required for most applications.
 */
class SystemStatusPlugin : public MavRosPlugin
{
public:
	SystemStatusPlugin() :
		hb_diag("Heartbeat", 10),
		mem_diag("APM Memory"),
		hwst_diag("APM Hardware"),
		sys_diag("System"),
		batt_diag("Battery")
	{};

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;
		diag_updater.add(hb_diag);
		diag_updater.add(sys_diag);
		diag_updater.add(batt_diag);
#ifdef MAVLINK_MSG_ID_MEMINFO
		diag_updater.add(mem_diag);
#endif
#ifdef MAVLINK_MSG_ID_HWSTATUS
		diag_updater.add(hwst_diag);
#endif

		double conn_timeout_d;
		double conn_heartbeat_d;
		double min_voltage;

		nh.param("conn_timeout", conn_timeout_d, 30.0);
		nh.param("conn_heartbeat", conn_heartbeat_d, 0.0);
		nh.param("sys/min_voltage", min_voltage, 6.0);

		batt_diag.set_min_voltage(min_voltage);

		conn_timeout = boost::posix_time::milliseconds(conn_timeout_d * 1000);
		timeout_timer.reset(new boost::asio::deadline_timer(uas->timer_service, conn_timeout));

		if (conn_heartbeat_d > 0.0) {
			conn_heartbeat = boost::posix_time::milliseconds(conn_heartbeat_d * 1000);
			heartbeat_timer.reset(new boost::asio::deadline_timer(uas->timer_service, conn_heartbeat));
			heartbeat_timer->async_wait(boost::bind(&SystemStatusPlugin::heartbeat_cb, this, _1));
		}

		state_pub = nh.advertise<mavros::State>("state", 10);
		batt_pub = nh.advertise<mavros::BatteryStatus>("battery", 10);
		rate_srv = nh.advertiseService("set_stream_rate", &SystemStatusPlugin::set_rate_cb, this);
	}

	const std::string get_name() const {
		return "SystemStatus";
	}

	const std::vector<uint8_t> get_supported_messages() const {
		return {
			MAVLINK_MSG_ID_HEARTBEAT,
			MAVLINK_MSG_ID_SYS_STATUS,
			MAVLINK_MSG_ID_STATUSTEXT
#ifdef MAVLINK_MSG_ID_MEMINFO
			, MAVLINK_MSG_ID_MEMINFO
#endif
#ifdef MAVLINK_MSG_ID_HWSTATUS
			, MAVLINK_MSG_ID_HWSTATUS
#endif
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:
			{
				mavlink_heartbeat_t hb;
				mavlink_msg_heartbeat_decode(msg, &hb);
				hb_diag.tick(hb);

				// update context && setup connection timeout
				uas->update_heartbeat(hb.type, hb.autopilot);
				uas->update_connection_status(true);
				timeout_timer->cancel();
				timeout_timer->expires_from_now(conn_timeout);
				timeout_timer->async_wait(boost::bind(&SystemStatusPlugin::timeout_cb, this, _1));

				mavros::StatePtr state_msg = boost::make_shared<mavros::State>();
				state_msg->header.stamp = ros::Time::now();
				state_msg->armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
				state_msg->guided = hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED;
				state_msg->mode = str_mode_v10(hb.base_mode, hb.custom_mode);

				state_pub.publish(state_msg);
			}
			break;

		case MAVLINK_MSG_ID_SYS_STATUS:
			{
				mavlink_sys_status_t stat;
				mavlink_msg_sys_status_decode(msg, &stat);

				float volt = stat.voltage_battery / 1000.0;	// mV
				float curr = stat.current_battery / 100.0;	// 10 mA or -1
				float rem = stat.battery_remaining / 100.0;	// or -1

				mavros::BatteryStatusPtr batt_msg = boost::make_shared<mavros::BatteryStatus>();
				batt_msg->header.stamp = ros::Time::now();
				batt_msg->voltage = volt;
				batt_msg->current = curr;
				batt_msg->remaining = rem;

				sys_diag.set(stat);
				batt_diag.set(volt, curr, rem);
				batt_pub.publish(batt_msg);
			}
			break;

		case MAVLINK_MSG_ID_STATUSTEXT:
			{
				mavlink_statustext_t textm;
				mavlink_msg_statustext_decode(msg, &textm);

				std::string text(textm.text,
						strnlen(textm.text, sizeof(textm.text)));

				if (uas->is_ardupilotmega())
					process_statustext_apm_quirk(textm.severity, text);
				else
					process_statustext_normal(textm.severity, text);
			}
			break;

		/* -*- APM additional messages -*- */

#ifdef MAVLINK_MSG_ID_MEMINFO
		case MAVLINK_MSG_ID_MEMINFO:
			{
				mavlink_meminfo_t mem;
				mavlink_msg_meminfo_decode(msg, &mem);
				mem_diag.set(mem.freemem, mem.brkval);
			}
			break;
#endif

#ifdef MAVLINK_MSG_ID_HWSTATUS
		case MAVLINK_MSG_ID_HWSTATUS:
			{
				mavlink_hwstatus_t hwst;
				mavlink_msg_hwstatus_decode(msg, &hwst);
				hwst_diag.set(hwst.Vcc, hwst.I2Cerr);
			}
			break;
#endif
		};
	}

private:
	HeartbeatStatus hb_diag;
	MemInfo mem_diag;
	HwStatus hwst_diag;
	SystemStatusDiag sys_diag;
	BatteryStatusDiag batt_diag;
	UAS *uas;
	boost::posix_time::time_duration conn_timeout;
	boost::posix_time::time_duration conn_heartbeat;
	std::unique_ptr<boost::asio::deadline_timer> timeout_timer;
	std::unique_ptr<boost::asio::deadline_timer> heartbeat_timer;

	ros::Publisher state_pub;
	ros::Publisher batt_pub;
	ros::ServiceServer rate_srv;

	void process_statustext_normal(uint8_t severity, std::string &text) {
		switch (severity) {
		case MAV_SEVERITY_EMERGENCY:
		case MAV_SEVERITY_ALERT:
		case MAV_SEVERITY_CRITICAL:
		case MAV_SEVERITY_ERROR:
			ROS_ERROR_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		case MAV_SEVERITY_WARNING:
		case MAV_SEVERITY_NOTICE:
			ROS_WARN_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		case MAV_SEVERITY_INFO:
			ROS_INFO_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		case MAV_SEVERITY_DEBUG:
		default:
			ROS_DEBUG_STREAM_NAMED("fcu", "FCU: " << text);
			break;
		};

	}

	void process_statustext_apm_quirk(uint8_t severity, std::string &text) {
		switch (severity) {
		case 1: // SEVERITY_LOW
			ROS_INFO_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		case 2: // SEVERITY_MEDIUM
			ROS_WARN_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		case 3: // SEVERITY_HIGH
		case 4: // SEVERITY_CRITICAL
		case 5: // SEVERITY_USER_RESPONSE
			ROS_ERROR_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		default:
			ROS_DEBUG_STREAM_NAMED("fcu", "FCU: UNK(" <<
					(int)severity << "): " << text);
			break;
		};

	}

	static std::string str_base_mode(int base_mode) {
		std::ostringstream mode;
		mode << "MODE(0x" << std::hex << std::uppercase << base_mode << ")";
		return mode.str();
	}

	static std::string str_custom_mode(int custom_mode) {
		std::ostringstream mode;
		mode << "CMODE(" << custom_mode << ")";
		return mode.str();
	}

	//! APM:Plane custom mode -> string
	static std::string str_mode_arduplane(int custom_mode) {
		switch (custom_mode) {
		case 0: return "MANUAL";	break;
		case 1: return "CIRCLE";	break;
		case 2: return "STABILIZE";	break;
		case 3: return "TRAINING";	break;
		case 4: return "ACRO";		break;
		case 5: return "FBWA";		break;
		case 6: return "FBWB";		break;
		case 7: return "CRUISE";	break;
		case 8: return "AUTOTUNE";	break;
		case 10: return "AUTO";		break;
		case 11: return "RTL";		break;
		case 12: return "LOITER";	break;
		case 14: return "LAND";		break;
		case 15: return "GUIDED";	break;
		case 16: return "INITIALISING";	break;
		default:
			 return str_custom_mode(custom_mode);
			 break;
		}
	};

	//! APM:Copter custom mode -> string
	static std::string str_mode_arducopter(int custom_mode) {
		switch (custom_mode) {
		case 0: return "STABILIZE";	break;
		case 1: return "ACRO";		break;
		case 2: return "ALT_HOLD";	break;
		case 3: return "AUTO";		break;
		case 4: return "GUIDED";	break;
		case 5: return "LOITER";	break;
		case 6: return "RTL";		break;
		case 7: return "CIRCLE";	break;
		case 8: return "POSITION";	break;
		case 9: return "LAND";		break;
		case 10: return "OF_LOITER";	break;
		case 11: return "APPROACH";	break;
		default:
			 return str_custom_mode(custom_mode);
			 break;
		}
	};

	//! Port pymavlink mavutil.mode_string_v10
	std::string str_mode_v10(int base_mode, int custom_mode) {

		if (!(base_mode && MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))
			return str_base_mode(base_mode);

		enum MAV_TYPE type = uas->get_type();
		if (uas->is_ardupilotmega()) {
			if (type == MAV_TYPE_QUADROTOR ||
					type == MAV_TYPE_HEXAROTOR ||
					type == MAV_TYPE_OCTOROTOR ||
					type == MAV_TYPE_TRICOPTER ||
					type == MAV_TYPE_COAXIAL)
				return str_mode_arducopter(custom_mode);
			else if (type == MAV_TYPE_FIXED_WING)
				return str_mode_arduplane(custom_mode);
			else
				/* TODO: APM:Rover */
				return str_custom_mode(custom_mode);
		}
		else
			/* TODO: other autopilot */
			return str_custom_mode(custom_mode);
	}

	void timeout_cb(boost::system::error_code error) {
		if (error)
			return;

		uas->update_connection_status(false);
	}

	void heartbeat_cb(boost::system::error_code error) {
		if (error)
			return;

		mavlink_message_t msg;
		mavlink_msg_heartbeat_pack_chan(UAS_PACK_CHAN(uas), &msg,
				MAV_TYPE_ONBOARD_CONTROLLER,
				MAV_AUTOPILOT_INVALID,
				MAV_MODE_MANUAL_ARMED,
				0,
				MAV_STATE_ACTIVE
				);

		uas->mav_link->send_message(&msg);

		/* restart timer */
		heartbeat_timer->expires_from_now(conn_heartbeat);
		heartbeat_timer->async_wait(boost::bind(&SystemStatusPlugin::heartbeat_cb, this, _1));
	}

	bool set_rate_cb(mavros::StreamRate::Request &req,
			mavros::StreamRate::Response &res) {

		mavlink_message_t msg;
		mavlink_msg_request_data_stream_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas),
				req.stream_id,
				req.message_rate,
				(req.on_off)? 1 : 0
				);

		uas->mav_link->send_message(&msg);
		return true;
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SystemStatusPlugin, mavplugin::MavRosPlugin)


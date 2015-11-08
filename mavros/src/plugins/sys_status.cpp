/**
 * @brief System Status plugin
 * @file sys_status.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013,2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>

namespace mavplugin {
/**
 * Heartbeat status publisher
 *
 * Based on diagnistic_updater::FrequencyStatus
 */
class HeartbeatStatus : public diagnostic_updater::DiagnosticTask
{
public:
	HeartbeatStatus(const std::string &name, size_t win_size) :
		diagnostic_updater::DiagnosticTask(name),
		times_(win_size),
		seq_nums_(win_size),
		window_size_(win_size),
		min_freq_(0.2),
		max_freq_(100),
		tolerance_(0.1),
		autopilot(MAV_AUTOPILOT_GENERIC),
		type(MAV_TYPE_GENERIC),
		system_status(MAV_STATE_UNINIT)
	{
		clear();
	}

	void clear() {
		lock_guard lock(mutex);
		ros::Time curtime = ros::Time::now();
		count_ = 0;

		for (size_t i = 0; i < window_size_; i++) {
			times_[i] = curtime;
			seq_nums_[i] = count_;
		}

		hist_indx_ = 0;
	}

	void tick(uint8_t type_, uint8_t autopilot_,
			std::string &mode_, uint8_t system_status_) {
		lock_guard lock(mutex);
		count_++;

		type = static_cast<enum MAV_TYPE>(type_);
		autopilot = static_cast<enum MAV_AUTOPILOT>(autopilot_);
		mode = mode_;
		system_status = static_cast<enum MAV_STATE>(system_status_);
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);
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

		stat.addf("Heartbeats since startup", "%d", count_);
		stat.addf("Frequency (Hz)", "%f", freq);
		stat.add("Vehicle type", mavros::UAS::str_type(type));
		stat.add("Autopilot type", mavros::UAS::str_autopilot(autopilot));
		stat.add("Mode", mode);
		stat.add("System status", mavros::UAS::str_system_status(system_status));
	}

private:
	int count_;
	std::vector<ros::Time> times_;
	std::vector<int> seq_nums_;
	int hist_indx_;
	std::recursive_mutex mutex;
	const size_t window_size_;
	const double min_freq_;
	const double max_freq_;
	const double tolerance_;

	enum MAV_AUTOPILOT autopilot;
	enum MAV_TYPE type;
	std::string mode;
	enum MAV_STATE system_status;
};


/**
 * @brief System status diagnostic updater
 */
class SystemStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
	SystemStatusDiag(const std::string &name) :
		diagnostic_updater::DiagnosticTask(name),
		last_st {}
	{ };

	void set(mavlink_sys_status_t &st) {
		lock_guard lock(mutex);
		last_st = st;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);

		if ((last_st.onboard_control_sensors_health & last_st.onboard_control_sensors_enabled)
				!= last_st.onboard_control_sensors_enabled)
			stat.summary(2, "Sensor helth");
		else
			stat.summary(0, "Normal");

		stat.addf("Sensor present", "0x%08X", last_st.onboard_control_sensors_present);
		stat.addf("Sensor enabled", "0x%08X", last_st.onboard_control_sensors_enabled);
		stat.addf("Sensor helth", "0x%08X", last_st.onboard_control_sensors_health);

		// decode sensor health mask
#define STAT_ADD_SENSOR(msg, sensor_mask)	\
	if (last_st.onboard_control_sensors_enabled & sensor_mask)	\
		stat.add(msg, (last_st.onboard_control_sensors_health & sensor_mask) ? "Ok" : "Fail")

		STAT_ADD_SENSOR("Sensor 3D Gyro", MAV_SYS_STATUS_SENSOR_3D_GYRO);
		STAT_ADD_SENSOR("Sensor 3D Accel", MAV_SYS_STATUS_SENSOR_3D_ACCEL);
		STAT_ADD_SENSOR("Sensor 3D Mag", MAV_SYS_STATUS_SENSOR_3D_MAG);
		STAT_ADD_SENSOR("Sensor Abs Pressure", MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE);
		STAT_ADD_SENSOR("Sensor Diff Pressure", MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
		STAT_ADD_SENSOR("Sensor GPS", MAV_SYS_STATUS_SENSOR_GPS);
		STAT_ADD_SENSOR("Sensor Optical Flow", MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
		STAT_ADD_SENSOR("Sensor Vision Position", MAV_SYS_STATUS_SENSOR_VISION_POSITION);
		STAT_ADD_SENSOR("Sensor Laser Position", MAV_SYS_STATUS_SENSOR_LASER_POSITION);
		STAT_ADD_SENSOR("Sensor Ext Grount Truth", MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
		STAT_ADD_SENSOR("Sensor Ang Rate Control", MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
		STAT_ADD_SENSOR("Sensor Attitude Stab", MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION);
		STAT_ADD_SENSOR("Sensor Yaw Position", MAV_SYS_STATUS_SENSOR_YAW_POSITION);
		STAT_ADD_SENSOR("Sensor Z/Alt Control", MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL);
		STAT_ADD_SENSOR("Sensor X/Y Pos Control", MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
		STAT_ADD_SENSOR("Sensor Motor Outputs", MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
		STAT_ADD_SENSOR("Sensor RC Receiver", MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
		STAT_ADD_SENSOR("Sensor 3D Gyro 2", MAV_SYS_STATUS_SENSOR_3D_GYRO2);
		STAT_ADD_SENSOR("Sensor 3D Accel 2", MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
		STAT_ADD_SENSOR("Sensor 3D Mag 2", MAV_SYS_STATUS_SENSOR_3D_MAG2);
		STAT_ADD_SENSOR("Geofence health", MAV_SYS_STATUS_GEOFENCE);
		STAT_ADD_SENSOR("AHRS health", MAV_SYS_STATUS_AHRS);
		STAT_ADD_SENSOR("Terrain health", MAV_SYS_STATUS_TERRAIN);

#undef STAT_ADD_SENSOR

		stat.addf("CPU Load (%)", "%.1f", last_st.load / 10.0);
		stat.addf("Drop rate (%)", "%.1f", last_st.drop_rate_comm / 10.0);
		stat.addf("Errors comm", "%d", last_st.errors_comm);
		stat.addf("Errors count #1", "%d", last_st.errors_count1);
		stat.addf("Errors count #2", "%d", last_st.errors_count2);
		stat.addf("Errors count #3", "%d", last_st.errors_count3);
		stat.addf("Errors count #4", "%d", last_st.errors_count4);
	}

private:
	std::recursive_mutex mutex;
	mavlink_sys_status_t last_st;
};


/**
 * @brief Battery diagnostic updater
 */
class BatteryStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
	BatteryStatusDiag(const std::string &name) :
		diagnostic_updater::DiagnosticTask(name),
		voltage(-1.0),
		current(0.0),
		remaining(0.0),
		min_voltage(6)
	{};

	void set_min_voltage(float volt) {
		lock_guard lock(mutex);
		min_voltage = volt;
	}

	void set(float volt, float curr, float rem) {
		lock_guard lock(mutex);
		voltage = volt;
		current = curr;
		remaining = rem;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);

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
	std::recursive_mutex mutex;
	float voltage;
	float current;
	float remaining;
	float min_voltage;
};


/**
 * @brief Memory usage diag (APM-only)
 */
class MemInfo : public diagnostic_updater::DiagnosticTask
{
public:
	MemInfo(const std::string &name) :
		diagnostic_updater::DiagnosticTask(name),
		freemem(-1),
		brkval(0)
	{};

	void set(uint16_t f, uint16_t b) {
		freemem = f;
		brkval = b;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		ssize_t freemem_ = freemem;
		uint16_t brkval_ = brkval;

		if (freemem < 0)
			stat.summary(2, "No data");
		else if (freemem < 200)
			stat.summary(1, "Low mem");
		else
			stat.summary(0, "Normal");

		stat.addf("Free memory (B)", "%zd", freemem_);
		stat.addf("Heap top", "0x%04X", brkval_);
	}

private:
	std::atomic<ssize_t> freemem;
	std::atomic<uint16_t> brkval;
};


/**
 * @brief Hardware status (APM-only)
 */
class HwStatus : public diagnostic_updater::DiagnosticTask
{
public:
	HwStatus(const std::string &name) :
		diagnostic_updater::DiagnosticTask(name),
		vcc(-1.0),
		i2cerr(0),
		i2cerr_last(0)
	{};

	void set(uint16_t v, uint8_t e) {
		lock_guard lock(mutex);
		vcc = v / 1000.0;
		i2cerr = e;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);

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
	std::recursive_mutex mutex;
	float vcc;
	size_t i2cerr;
	size_t i2cerr_last;
};


/**
 * @brief System status plugin.
 *
 * Required by all plugins.
 */
class SystemStatusPlugin : public MavRosPlugin
{
public:
	SystemStatusPlugin() :
		nh("~"),
		uas(nullptr),
		hb_diag("Heartbeat", 10),
		mem_diag("APM Memory"),
		hwst_diag("APM Hardware"),
		sys_diag("System"),
		batt_diag("Battery"),
		version_retries(RETRIES_COUNT),
		disable_diag(false)
	{};

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		ros::Duration conn_heartbeat;

		double conn_timeout_d;
		double conn_heartbeat_d;
		double min_voltage;

		nh.param("conn/timeout", conn_timeout_d, 30.0);
		nh.param("sys/min_voltage", min_voltage, 6.0);
		nh.param("sys/disable_diag", disable_diag, false);

		// rate parameter
		if (nh.getParam("conn/heartbeat_rate", conn_heartbeat_d) && conn_heartbeat_d != 0.0) {
			conn_heartbeat = ros::Duration(ros::Rate(conn_heartbeat_d));
		}
		else if (nh.getParam("conn/heartbeat", conn_heartbeat_d)) {
			// XXX deprecated parameter
			ROS_WARN_NAMED("sys", "SYS: parameter `~conn/heartbeat` deprecated, "
					"please use `~conn/heartbeat_rate` instead!");
			conn_heartbeat = ros::Duration(conn_heartbeat_d);
		}

		// heartbeat diag always enabled
		UAS_DIAG(uas).add(hb_diag);
		if (!disable_diag) {
			UAS_DIAG(uas).add(sys_diag);
			UAS_DIAG(uas).add(batt_diag);

			batt_diag.set_min_voltage(min_voltage);
		}


		// one-shot timeout timer
		timeout_timer = nh.createTimer(ros::Duration(conn_timeout_d),
				&SystemStatusPlugin::timeout_cb, this, true);
		timeout_timer.start();

		if (!conn_heartbeat.isZero()) {
			heartbeat_timer = nh.createTimer(conn_heartbeat,
					&SystemStatusPlugin::heartbeat_cb, this);
			heartbeat_timer.start();
		}

		// version request timer
		autopilot_version_timer = nh.createTimer(ros::Duration(1.0),
				&SystemStatusPlugin::autopilot_version_cb, this);
		autopilot_version_timer.stop();

		// subscribe to connection event
		uas->sig_connection_changed.connect(boost::bind(&SystemStatusPlugin::connection_cb, this, _1));

		state_pub = nh.advertise<mavros_msgs::State>("state", 10, true);
		extended_state_pub = nh.advertise<mavros_msgs::ExtendedState>("extended_state", 10);
		batt_pub = nh.advertise<mavros_msgs::BatteryStatus>("battery", 10);
		rate_srv = nh.advertiseService("set_stream_rate", &SystemStatusPlugin::set_rate_cb, this);
		mode_srv = nh.advertiseService("set_mode", &SystemStatusPlugin::set_mode_cb, this);

		// init state topic
		publish_disconnection();
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_HEARTBEAT, &SystemStatusPlugin::handle_heartbeat),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_SYS_STATUS, &SystemStatusPlugin::handle_sys_status),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_STATUSTEXT, &SystemStatusPlugin::handle_statustext),
#ifdef MAVLINK_MSG_ID_MEMINFO
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_MEMINFO, &SystemStatusPlugin::handle_meminfo),
#endif
#ifdef MAVLINK_MSG_ID_HWSTATUS
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_HWSTATUS, &SystemStatusPlugin::handle_hwstatus),
#endif
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_AUTOPILOT_VERSION, &SystemStatusPlugin::handle_autopilot_version),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_EXTENDED_SYS_STATE, &SystemStatusPlugin::handle_extended_sys_state),
		};
	}

private:
	ros::NodeHandle nh;
	UAS *uas;

	HeartbeatStatus hb_diag;
	MemInfo mem_diag;
	HwStatus hwst_diag;
	SystemStatusDiag sys_diag;
	BatteryStatusDiag batt_diag;
	ros::Timer timeout_timer;
	ros::Timer heartbeat_timer;
	ros::Timer autopilot_version_timer;

	ros::Publisher state_pub;
	ros::Publisher extended_state_pub;
	ros::Publisher batt_pub;
	ros::ServiceServer rate_srv;
	ros::ServiceServer mode_srv;

	static constexpr int RETRIES_COUNT = 6;
	int version_retries;
	bool disable_diag;

	/* -*- mid-level helpers -*- */

	/**
	 * Sent STATUSTEXT message to rosout
	 *
	 * @param[in] severity  Levels defined in common.xml
	 */
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
			ROS_DEBUG_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		default:
			ROS_WARN_STREAM_NAMED("fcu", "FCU: UNK(" <<
					int(severity) << "): " << text);
			break;
		};
	}

	static inline std::string custom_version_to_hex_string(uint8_t array[8])
	{
		// inefficient, but who care for one time call function?

		std::ostringstream ss;
		ss << std::setfill('0');

		for (ssize_t i = 7; i >= 0; i--)
			ss << std::hex << std::setw(2) << int(array[i]);

		return ss.str();
	}

	void process_autopilot_version_normal(mavlink_autopilot_version_t &apv, uint8_t sysid, uint8_t compid)
	{
		char prefix[16];
		::snprintf(prefix, sizeof(prefix) - 1, "VER: %d.%d", sysid, compid);

		ROS_INFO_NAMED("sys", "%s: Capabilities 0x%016llx", prefix, (long long int)apv.capabilities);
		ROS_INFO_NAMED("sys", "%s: Flight software:     %08x (%s)",
				prefix,
				apv.flight_sw_version,
				custom_version_to_hex_string(apv.flight_custom_version).c_str());
		ROS_INFO_NAMED("sys", "%s: Middleware software: %08x (%s)",
				prefix,
				apv.middleware_sw_version,
				custom_version_to_hex_string(apv.middleware_custom_version).c_str());
		ROS_INFO_NAMED("sys", "%s: OS software:         %08x (%s)",
				prefix,
				apv.os_sw_version,
				custom_version_to_hex_string(apv.os_custom_version).c_str());
		ROS_INFO_NAMED("sys", "%s: Board hardware:      %08x", prefix, apv.board_version);
		ROS_INFO_NAMED("sys", "%s: VID/PID: %04x:%04x", prefix, apv.vendor_id, apv.product_id);
		ROS_INFO_NAMED("sys", "%s: UID: %016llx", prefix, (long long int)apv.uid);
	}

	void process_autopilot_version_apm_quirk(mavlink_autopilot_version_t &apv, uint8_t sysid, uint8_t compid)
	{
		char prefix[16];
		::snprintf(prefix, sizeof(prefix) - 1, "VER: %d.%d", sysid, compid);

		// Note based on current APM's impl.
		// APM uses custom version array[8] as a string
		ROS_INFO_NAMED("sys", "%s: Capabilities 0x%016llx", prefix, (long long int)apv.capabilities);
		ROS_INFO_NAMED("sys", "%s: Flight software:     %08x (%*s)",
				prefix,
				apv.flight_sw_version,
				8, apv.flight_custom_version);
		ROS_INFO_NAMED("sys", "%s: Middleware software: %08x (%*s)",
				prefix,
				apv.middleware_sw_version,
				8, apv.middleware_custom_version);
		ROS_INFO_NAMED("sys", "%s: OS software:         %08x (%*s)",
				prefix,
				apv.os_sw_version,
				8, apv.os_custom_version);
		ROS_INFO_NAMED("sys", "%s: Board hardware:      %08x", prefix, apv.board_version);
		ROS_INFO_NAMED("sys", "%s: VID/PID: %04x:%04x", prefix, apv.vendor_id, apv.product_id);
		ROS_INFO_NAMED("sys", "%s: UID: %016llx", prefix, (long long int)apv.uid);
	}

	void publish_disconnection() {
		auto state_msg = boost::make_shared<mavros_msgs::State>();
		state_msg->header.stamp = ros::Time::now();
		state_msg->connected = false;
		state_msg->armed = false;
		state_msg->guided = false;
		state_msg->mode = "";

		state_pub.publish(state_msg);
	}

	/* -*- message handlers -*- */

	void handle_heartbeat(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (!uas->is_my_target(sysid)) {
			ROS_DEBUG_NAMED("sys", "HEARTBEAT from [%d, %d] dropped.", sysid, compid);
			return;
		}

		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);

		// update context && setup connection timeout
		uas->update_heartbeat(hb.type, hb.autopilot, hb.base_mode);
		uas->update_connection_status(true);
		timeout_timer.stop();
		timeout_timer.start();

		// build state message after updating uas
		auto state_msg = boost::make_shared<mavros_msgs::State>();
		state_msg->header.stamp = ros::Time::now();
		state_msg->connected = true;
		state_msg->armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
		state_msg->guided = hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED;
		state_msg->mode = uas->str_mode_v10(hb.base_mode, hb.custom_mode);

		state_pub.publish(state_msg);
		hb_diag.tick(hb.type, hb.autopilot, state_msg->mode, hb.system_status);
	}

	void handle_extended_sys_state(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_extended_sys_state_t state;
		mavlink_msg_extended_sys_state_decode(msg, &state);

		auto state_msg = boost::make_shared<mavros_msgs::ExtendedState>();
		state_msg->header.stamp = ros::Time::now();
		state_msg->vtol_state = state.vtol_state;
		state_msg->landed_state = state.landed_state;

		extended_state_pub.publish(state_msg);
	}

	void handle_sys_status(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_sys_status_t stat;
		mavlink_msg_sys_status_decode(msg, &stat);

		float volt = stat.voltage_battery / 1000.0f;	// mV
		float curr = stat.current_battery / 100.0f;	// 10 mA or -1
		float rem = stat.battery_remaining / 100.0f;	// or -1

		auto batt_msg = boost::make_shared<mavros_msgs::BatteryStatus>();
		batt_msg->header.stamp = ros::Time::now();
		batt_msg->voltage = volt;
		batt_msg->current = curr;
		batt_msg->remaining = rem;

		sys_diag.set(stat);
		batt_diag.set(volt, curr, rem);
		batt_pub.publish(batt_msg);
	}

	void handle_statustext(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_statustext_t textm;
		mavlink_msg_statustext_decode(msg, &textm);

		std::string text(textm.text,
				strnlen(textm.text, sizeof(textm.text)));

		process_statustext_normal(textm.severity, text);
	}

#ifdef MAVLINK_MSG_ID_MEMINFO
	void handle_meminfo(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_meminfo_t mem;
		mavlink_msg_meminfo_decode(msg, &mem);
		mem_diag.set(mem.freemem, mem.brkval);
	}
#endif

#ifdef MAVLINK_MSG_ID_HWSTATUS
	void handle_hwstatus(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_hwstatus_t hwst;
		mavlink_msg_hwstatus_decode(msg, &hwst);
		hwst_diag.set(hwst.Vcc, hwst.I2Cerr);
	}
#endif

	void handle_autopilot_version(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_autopilot_version_t apv;
		mavlink_msg_autopilot_version_decode(msg, &apv);

		// we want to store only FCU caps
		if (uas->is_my_target(sysid, compid)) {
			autopilot_version_timer.stop();
			uas->update_capabilities(true, apv.capabilities);
		}

		// but print all version responses
		if (uas->is_ardupilotmega())
			process_autopilot_version_apm_quirk(apv, sysid, compid);
		else
			process_autopilot_version_normal(apv, sysid, compid);
	}

	/* -*- timer callbacks -*- */

	void timeout_cb(const ros::TimerEvent &event) {
		uas->update_connection_status(false);
	}

	void heartbeat_cb(const ros::TimerEvent &event) {
		mavlink_message_t msg;
		mavlink_msg_heartbeat_pack_chan(UAS_PACK_CHAN(uas), &msg,
				MAV_TYPE_ONBOARD_CONTROLLER,
				MAV_AUTOPILOT_INVALID,
				MAV_MODE_MANUAL_ARMED,
				0,
				MAV_STATE_ACTIVE
				);

		UAS_FCU(uas)->send_message(&msg);
	}

	void autopilot_version_cb(const ros::TimerEvent &event) {
		bool ret = false;

		// Request from all first 3 times, then fallback to unicast
		bool do_broadcast = version_retries > RETRIES_COUNT / 2;

		try {
			auto client = nh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

			mavros_msgs::CommandLong cmd{};

			cmd.request.broadcast = do_broadcast;
			cmd.request.command = MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
			cmd.request.confirmation = false;
			cmd.request.param1 = 1.0;

			ROS_DEBUG_NAMED("sys", "VER: Sending %s request.",
					(do_broadcast) ? "broadcast" : "unicast");
			ret = client.call(cmd);
		}
		catch (ros::InvalidNameException &ex) {
			ROS_ERROR_NAMED("sys", "VER: %s", ex.what());
		}

		ROS_ERROR_COND_NAMED(!ret, "sys", "VER: command plugin service call failed!");

		if (version_retries > 0) {
			version_retries--;
			ROS_WARN_COND_NAMED(version_retries != RETRIES_COUNT - 1, "sys",
					"VER: %s request timeout, retries left %d",
					(do_broadcast) ? "broadcast" : "unicast",
					version_retries);
		}
		else {
			uas->update_capabilities(false);
			autopilot_version_timer.stop();
			ROS_WARN_NAMED("sys", "VER: your FCU don't support AUTOPILOT_VERSION, "
					"switched to default capabilities");
		}
	}

	void connection_cb(bool connected) {
		// if connection changes, start delayed version request
		version_retries = RETRIES_COUNT;
		if (connected)
			autopilot_version_timer.start();
		else
			autopilot_version_timer.stop();

		// add/remove APM diag tasks
		if (connected && disable_diag && uas->is_ardupilotmega()) {
#ifdef MAVLINK_MSG_ID_MEMINFO
			UAS_DIAG(uas).add(mem_diag);
#endif
#ifdef MAVLINK_MSG_ID_HWSTATUS
			UAS_DIAG(uas).add(hwst_diag);
#endif
#if !defined(MAVLINK_MSG_ID_MEMINFO) || !defined(MAVLINK_MSG_ID_HWSTATUS)
			ROS_INFO_NAMED("sys", "SYS: APM detected, but mavros uses different dialect. "
					"Extra diagnostic disabled.");
#else
			ROS_DEBUG_NAMED("sys", "SYS: APM extra diagnostics enabled.");
#endif
		}
		else {
			UAS_DIAG(uas).removeByName(mem_diag.getName());
			UAS_DIAG(uas).removeByName(hwst_diag.getName());
			ROS_DEBUG_NAMED("sys", "SYS: APM extra diagnostics disabled.");
		}

		if (!connected) {
			// publish connection change
			publish_disconnection();
		}
	}

	/* -*- ros callbacks -*- */

	bool set_rate_cb(mavros_msgs::StreamRate::Request &req,
			mavros_msgs::StreamRate::Response &res) {
		mavlink_message_t msg;
		mavlink_msg_request_data_stream_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas),
				req.stream_id,
				req.message_rate,
				(req.on_off) ? 1 : 0
				);

		UAS_FCU(uas)->send_message(&msg);
		return true;
	}

	bool set_mode_cb(mavros_msgs::SetMode::Request &req,
			mavros_msgs::SetMode::Response &res) {
		mavlink_message_t msg;
		uint8_t base_mode = req.base_mode;
		uint32_t custom_mode = 0;

		if (req.custom_mode != "") {
			if (!uas->cmode_from_str(req.custom_mode, custom_mode)) {
				res.success = false;
				return true;
			}

			/**
			 * @note That call may trigger unexpected arming change because
			 *       base_mode arming flag state based on previous HEARTBEAT
			 *       message value.
			 */
			base_mode |= (uas->get_armed()) ? MAV_MODE_FLAG_SAFETY_ARMED : 0;
			base_mode |= (uas->get_hil_state()) ? MAV_MODE_FLAG_HIL_ENABLED : 0;
			base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
		}

		mavlink_msg_set_mode_pack_chan(UAS_PACK_CHAN(uas), &msg,
				uas->get_tgt_system(),
				base_mode,
				custom_mode);
		UAS_FCU(uas)->send_message(&msg);
		res.success = true;
		return true;
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SystemStatusPlugin, mavplugin::MavRosPlugin)


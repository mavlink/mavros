/**
 * @brief System Status plugin
 * @file sys_status.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/VehicleInfo.h>
#include <mavros_msgs/VehicleInfoGet.h>
#include <mavros_msgs/MessageInterval.h>


#ifdef HAVE_SENSOR_MSGS_BATTERYSTATE_MSG
#include <sensor_msgs/BatteryState.h>
using BatteryMsg = sensor_msgs::BatteryState;
#else
#include <mavros_msgs/BatteryStatus.h>
using BatteryMsg = mavros_msgs::BatteryStatus;
#endif

namespace mavros {
namespace std_plugins {
using mavlink::minimal::MAV_TYPE;
using mavlink::minimal::MAV_AUTOPILOT;
using mavlink::minimal::MAV_STATE;
using utils::enum_value;

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
		autopilot(MAV_AUTOPILOT::GENERIC),
		type(MAV_TYPE::GENERIC),
		system_status(MAV_STATE::UNINIT)
	{
		clear();
	}

	void clear()
	{
		std::lock_guard<std::mutex> lock(mutex);
		ros::Time curtime = ros::Time::now();
		count_ = 0;

		for (size_t i = 0; i < window_size_; i++) {
			times_[i] = curtime;
			seq_nums_[i] = count_;
		}

		hist_indx_ = 0;
	}

	void tick(uint8_t type_, uint8_t autopilot_,
			std::string &mode_, uint8_t system_status_)
	{
		std::lock_guard<std::mutex> lock(mutex);
		count_++;

		type = static_cast<MAV_TYPE>(type_);
		autopilot = static_cast<MAV_AUTOPILOT>(autopilot_);
		mode = mode_;
		system_status = static_cast<MAV_STATE>(system_status_);
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		std::lock_guard<std::mutex> lock(mutex);

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
		stat.add("Vehicle type", utils::to_string(type));
		stat.add("Autopilot type", utils::to_string(autopilot));
		stat.add("Mode", mode);
		stat.add("System status", utils::to_string(system_status));
	}

private:
	int count_;
	std::vector<ros::Time> times_;
	std::vector<int> seq_nums_;
	int hist_indx_;
	std::mutex mutex;
	const size_t window_size_;
	const double min_freq_;
	const double max_freq_;
	const double tolerance_;

	MAV_AUTOPILOT autopilot;
	MAV_TYPE type;
	std::string mode;
	MAV_STATE system_status;
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
	{ }

	void set(mavlink::common::msg::SYS_STATUS &st)
	{
		std::lock_guard<std::mutex> lock(mutex);
		last_st = st;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		std::lock_guard<std::mutex> lock(mutex);

		if ((last_st.onboard_control_sensors_health & last_st.onboard_control_sensors_enabled)
				!= last_st.onboard_control_sensors_enabled)
			stat.summary(2, "Sensor health");
		else
			stat.summary(0, "Normal");

		stat.addf("Sensor present", "0x%08X", last_st.onboard_control_sensors_present);
		stat.addf("Sensor enabled", "0x%08X", last_st.onboard_control_sensors_enabled);
		stat.addf("Sensor health", "0x%08X", last_st.onboard_control_sensors_health);

		using STS = mavlink::common::MAV_SYS_STATUS_SENSOR;

		// [[[cog:
		// import pymavlink.dialects.v20.common as common
		// ename = 'MAV_SYS_STATUS_SENSOR'
		// ename_pfx2 = 'MAV_SYS_STATUS_'
		//
		// enum = sorted(common.enums[ename].items())
		// enum.pop() # -> remove ENUM_END
		//
		// for k, e in enum:
		//     desc = e.description.split(' ', 1)[1] if e.description.startswith('0x') else e.description
		//     sts = e.name
		//
		//     if sts.startswith(ename + '_'):
		//         sts = sts[len(ename) + 1:]
		//     if sts.startswith(ename_pfx2):
		//         sts = sts[len(ename_pfx2):]
		//     if sts[0].isdigit():
		//         sts = 'SENSOR_' + sts
		//
		//     cog.outl(f"""\
		//     if (last_st.onboard_control_sensors_enabled & enum_value(STS::{sts}))
		//     \tstat.add("{desc.strip()}", (last_st.onboard_control_sensors_health & enum_value(STS::{sts})) ? "Ok" : "Fail");""")
		// ]]]
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::SENSOR_3D_GYRO))
			stat.add("3D gyro", (last_st.onboard_control_sensors_health & enum_value(STS::SENSOR_3D_GYRO)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::SENSOR_3D_ACCEL))
			stat.add("3D accelerometer", (last_st.onboard_control_sensors_health & enum_value(STS::SENSOR_3D_ACCEL)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::SENSOR_3D_MAG))
			stat.add("3D magnetometer", (last_st.onboard_control_sensors_health & enum_value(STS::SENSOR_3D_MAG)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::ABSOLUTE_PRESSURE))
			stat.add("absolute pressure", (last_st.onboard_control_sensors_health & enum_value(STS::ABSOLUTE_PRESSURE)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::DIFFERENTIAL_PRESSURE))
			stat.add("differential pressure", (last_st.onboard_control_sensors_health & enum_value(STS::DIFFERENTIAL_PRESSURE)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::GPS))
			stat.add("GPS", (last_st.onboard_control_sensors_health & enum_value(STS::GPS)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::OPTICAL_FLOW))
			stat.add("optical flow", (last_st.onboard_control_sensors_health & enum_value(STS::OPTICAL_FLOW)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::VISION_POSITION))
			stat.add("computer vision position", (last_st.onboard_control_sensors_health & enum_value(STS::VISION_POSITION)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::LASER_POSITION))
			stat.add("laser based position", (last_st.onboard_control_sensors_health & enum_value(STS::LASER_POSITION)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::EXTERNAL_GROUND_TRUTH))
			stat.add("external ground truth (Vicon or Leica)", (last_st.onboard_control_sensors_health & enum_value(STS::EXTERNAL_GROUND_TRUTH)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::ANGULAR_RATE_CONTROL))
			stat.add("3D angular rate control", (last_st.onboard_control_sensors_health & enum_value(STS::ANGULAR_RATE_CONTROL)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::ATTITUDE_STABILIZATION))
			stat.add("attitude stabilization", (last_st.onboard_control_sensors_health & enum_value(STS::ATTITUDE_STABILIZATION)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::YAW_POSITION))
			stat.add("yaw position", (last_st.onboard_control_sensors_health & enum_value(STS::YAW_POSITION)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::Z_ALTITUDE_CONTROL))
			stat.add("z/altitude control", (last_st.onboard_control_sensors_health & enum_value(STS::Z_ALTITUDE_CONTROL)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::XY_POSITION_CONTROL))
			stat.add("x/y position control", (last_st.onboard_control_sensors_health & enum_value(STS::XY_POSITION_CONTROL)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::MOTOR_OUTPUTS))
			stat.add("motor outputs / control", (last_st.onboard_control_sensors_health & enum_value(STS::MOTOR_OUTPUTS)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::RC_RECEIVER))
			stat.add("rc receiver", (last_st.onboard_control_sensors_health & enum_value(STS::RC_RECEIVER)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::SENSOR_3D_GYRO2))
			stat.add("2nd 3D gyro", (last_st.onboard_control_sensors_health & enum_value(STS::SENSOR_3D_GYRO2)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::SENSOR_3D_ACCEL2))
			stat.add("2nd 3D accelerometer", (last_st.onboard_control_sensors_health & enum_value(STS::SENSOR_3D_ACCEL2)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::SENSOR_3D_MAG2))
			stat.add("2nd 3D magnetometer", (last_st.onboard_control_sensors_health & enum_value(STS::SENSOR_3D_MAG2)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::GEOFENCE))
			stat.add("geofence", (last_st.onboard_control_sensors_health & enum_value(STS::GEOFENCE)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::AHRS))
			stat.add("AHRS subsystem health", (last_st.onboard_control_sensors_health & enum_value(STS::AHRS)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::TERRAIN))
			stat.add("Terrain subsystem health", (last_st.onboard_control_sensors_health & enum_value(STS::TERRAIN)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::REVERSE_MOTOR))
			stat.add("Motors are reversed", (last_st.onboard_control_sensors_health & enum_value(STS::REVERSE_MOTOR)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::LOGGING))
			stat.add("Logging", (last_st.onboard_control_sensors_health & enum_value(STS::LOGGING)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::BATTERY))
			stat.add("Battery", (last_st.onboard_control_sensors_health & enum_value(STS::BATTERY)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::PROXIMITY))
			stat.add("Proximity", (last_st.onboard_control_sensors_health & enum_value(STS::PROXIMITY)) ? "Ok" : "Fail");
		if (last_st.onboard_control_sensors_enabled & enum_value(STS::SATCOM))
			stat.add("Satellite Communication", (last_st.onboard_control_sensors_health & enum_value(STS::SATCOM)) ? "Ok" : "Fail");
		// [[[end]]] (checksum: 890cfdc6d3b776c38a59b39f80ec7351)

		stat.addf("CPU Load (%)", "%.1f", last_st.load / 10.0);
		stat.addf("Drop rate (%)", "%.1f", last_st.drop_rate_comm / 10.0);
		stat.addf("Errors comm", "%d", last_st.errors_comm);
		stat.addf("Errors count #1", "%d", last_st.errors_count1);
		stat.addf("Errors count #2", "%d", last_st.errors_count2);
		stat.addf("Errors count #3", "%d", last_st.errors_count3);
		stat.addf("Errors count #4", "%d", last_st.errors_count4);
	}

private:
	std::mutex mutex;
	mavlink::common::msg::SYS_STATUS last_st;
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
	{ }

	void set_min_voltage(float volt) {
		std::lock_guard<std::mutex> lock(mutex);
		min_voltage = volt;
	}

	void set(float volt, float curr, float rem) {
		std::lock_guard<std::mutex> lock(mutex);
		voltage = volt;
		current = curr;
		remaining = rem;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		std::lock_guard<std::mutex> lock(mutex);

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
	std::mutex mutex;
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
	{ }

	void set(uint16_t f, uint16_t b) {
		freemem = f;
		brkval = b;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
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
	{ }

	void set(uint16_t v, uint8_t e) {
		std::lock_guard<std::mutex> lock(mutex);
		vcc = v / 1000.0;
		i2cerr = e;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		std::lock_guard<std::mutex> lock(mutex);

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
	std::mutex mutex;
	float vcc;
	size_t i2cerr;
	size_t i2cerr_last;
};


/**
 * @brief System status plugin.
 *
 * Required by all plugins.
 */
class SystemStatusPlugin : public plugin::PluginBase
{
public:
	SystemStatusPlugin() : PluginBase(),
		nh("~"),
		hb_diag("Heartbeat", 10),
		mem_diag("APM Memory"),
		hwst_diag("APM Hardware"),
		sys_diag("System"),
		batt_diag("Battery"),
		version_retries(RETRIES_COUNT),
		disable_diag(false),
		has_battery_status(false),
		battery_voltage(0.0),
		conn_heartbeat_mav_type(MAV_TYPE::ONBOARD_CONTROLLER)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		ros::Duration conn_heartbeat;

		double conn_timeout_d;
		double conn_heartbeat_d;
		double min_voltage;
		std::string conn_heartbeat_mav_type_str;

		nh.param("conn/timeout", conn_timeout_d, 10.0);
		nh.param("sys/min_voltage", min_voltage, 10.0);
		nh.param("sys/disable_diag", disable_diag, false);

		// heartbeat rate parameter
		if (nh.getParam("conn/heartbeat_rate", conn_heartbeat_d) && conn_heartbeat_d != 0.0) {
			conn_heartbeat = ros::Duration(ros::Rate(conn_heartbeat_d));
		}

		// heartbeat mav type parameter
		if (nh.getParam("conn/heartbeat_mav_type", conn_heartbeat_mav_type_str)) {
			conn_heartbeat_mav_type = utils::mav_type_from_str(conn_heartbeat_mav_type_str);
		}

		// heartbeat diag always enabled
		UAS_DIAG(m_uas).add(hb_diag);
		if (!disable_diag) {
			UAS_DIAG(m_uas).add(sys_diag);
			UAS_DIAG(m_uas).add(batt_diag);

			batt_diag.set_min_voltage(min_voltage);
		}


		// one-shot timeout timer
		timeout_timer = nh.createTimer(ros::Duration(conn_timeout_d),
				&SystemStatusPlugin::timeout_cb, this, true);
		//timeout_timer.start();

		if (!conn_heartbeat.isZero()) {
			heartbeat_timer = nh.createTimer(conn_heartbeat,
					&SystemStatusPlugin::heartbeat_cb, this);
			//heartbeat_timer.start();
		}

		// version request timer
		autopilot_version_timer = nh.createTimer(ros::Duration(1.0),
				&SystemStatusPlugin::autopilot_version_cb, this);
		autopilot_version_timer.stop();

		state_pub = nh.advertise<mavros_msgs::State>("state", 10, true);
		extended_state_pub = nh.advertise<mavros_msgs::ExtendedState>("extended_state", 10);
		batt_pub = nh.advertise<BatteryMsg>("battery", 10);
		estimator_status_pub = nh.advertise<mavros_msgs::EstimatorStatus>("estimator_status", 10);
		statustext_pub = nh.advertise<mavros_msgs::StatusText>("statustext/recv", 10);
		statustext_sub = nh.subscribe("statustext/send", 10, &SystemStatusPlugin::statustext_cb, this);
		rate_srv = nh.advertiseService("set_stream_rate", &SystemStatusPlugin::set_rate_cb, this);
		mode_srv = nh.advertiseService("set_mode", &SystemStatusPlugin::set_mode_cb, this);
		vehicle_info_get_srv = nh.advertiseService("vehicle_info_get", &SystemStatusPlugin::vehicle_info_get_cb, this);
		message_interval_srv = nh.advertiseService("set_message_interval", &SystemStatusPlugin::set_message_interval_cb, this);

		// init state topic
		publish_disconnection();
		enable_connection_cb();
	}

	Subscriptions get_subscriptions() {
		return {
			make_handler(&SystemStatusPlugin::handle_heartbeat),
			make_handler(&SystemStatusPlugin::handle_sys_status),
			make_handler(&SystemStatusPlugin::handle_statustext),
			make_handler(&SystemStatusPlugin::handle_meminfo),
			make_handler(&SystemStatusPlugin::handle_hwstatus),
			make_handler(&SystemStatusPlugin::handle_autopilot_version),
			make_handler(&SystemStatusPlugin::handle_extended_sys_state),
			make_handler(&SystemStatusPlugin::handle_battery_status),
			make_handler(&SystemStatusPlugin::handle_estimator_status),
		};
	}

private:
	ros::NodeHandle nh;

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
	ros::Publisher estimator_status_pub;
	ros::Publisher statustext_pub;
	ros::Subscriber statustext_sub;
	ros::ServiceServer rate_srv;
	ros::ServiceServer mode_srv;
	ros::ServiceServer vehicle_info_get_srv;
	ros::ServiceServer message_interval_srv;

	MAV_TYPE conn_heartbeat_mav_type;
	static constexpr int RETRIES_COUNT = 6;
	int version_retries;
	bool disable_diag;
	bool has_battery_status;
	float battery_voltage;

	using M_VehicleInfo = std::unordered_map<uint16_t, mavros_msgs::VehicleInfo>;
	M_VehicleInfo vehicles;

	/* -*- mid-level helpers -*- */

	// Get vehicle key for the unordered map containing all vehicles
	inline uint16_t get_vehicle_key(uint8_t sysid,uint8_t compid) {
		return sysid << 8 | compid;
	}

	// Find or create vehicle info
	inline M_VehicleInfo::iterator find_or_create_vehicle_info(uint8_t sysid, uint8_t compid) {
		auto key = get_vehicle_key(sysid, compid);
		M_VehicleInfo::iterator ret = vehicles.find(key);

		if (ret == vehicles.end()) {
			// Not found
			mavros_msgs::VehicleInfo v;
			v.sysid = sysid;
			v.compid = compid;
			v.available_info = 0;

			auto res = vehicles.emplace(key, v);	//-> pair<iterator, bool>
			ret = res.first;
		}

		ROS_ASSERT(ret != vehicles.end());
		return ret;
	}

	/**
	 * Sent STATUSTEXT message to rosout
	 *
	 * @param[in] severity  Levels defined in common.xml
	 */
	void process_statustext_normal(uint8_t severity, std::string &text)
	{
		using mavlink::common::MAV_SEVERITY;

		switch (severity) {
		// [[[cog:
		// for l1, l2 in (
		//     (('EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR'), 'ERROR'),
		//     (('WARNING', 'NOTICE'), 'WARN'),
		//     (('INFO', ), 'INFO'),
		//     (('DEBUG', ), 'DEBUG')
		//     ):
		//     for v in l1:
		//         cog.outl("case enum_value(MAV_SEVERITY::%s):" % v)
		//     cog.outl("\tROS_%s_STREAM_NAMED(\"fcu\", \"FCU: \" << text);" % l2)
		//     cog.outl("\tbreak;")
		// ]]]
		case enum_value(MAV_SEVERITY::EMERGENCY):
		case enum_value(MAV_SEVERITY::ALERT):
		case enum_value(MAV_SEVERITY::CRITICAL):
		case enum_value(MAV_SEVERITY::ERROR):
			ROS_ERROR_STREAM_NAMED("fcu", "FCU: " << text);
			break;
		case enum_value(MAV_SEVERITY::WARNING):
		case enum_value(MAV_SEVERITY::NOTICE):
			ROS_WARN_STREAM_NAMED("fcu", "FCU: " << text);
			break;
		case enum_value(MAV_SEVERITY::INFO):
			ROS_INFO_STREAM_NAMED("fcu", "FCU: " << text);
			break;
		case enum_value(MAV_SEVERITY::DEBUG):
			ROS_DEBUG_STREAM_NAMED("fcu", "FCU: " << text);
			break;
		// [[[end]]] (checksum: 315aa363b5ecb4dda66cc8e1e3d3aa48)
		default:
			ROS_WARN_STREAM_NAMED("fcu", "FCU: UNK(" << +severity << "): " << text);
			break;
		};
	}

	static std::string custom_version_to_hex_string(std::array<uint8_t, 8> &array)
	{
		// should be little-endian
		uint64_t b;
		memcpy(&b, array.data(), sizeof(uint64_t));
		b = le64toh(b);

		return utils::format("%016llx", b);
	}

	void process_autopilot_version_normal(mavlink::common::msg::AUTOPILOT_VERSION &apv, uint8_t sysid, uint8_t compid)
	{
		char prefix[16];
		std::snprintf(prefix, sizeof(prefix), "VER: %d.%d", sysid, compid);

		ROS_INFO_NAMED("sys", "%s: Capabilities         0x%016llx", prefix, (long long int)apv.capabilities);
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
		ROS_INFO_NAMED("sys", "%s: VID/PID:             %04x:%04x", prefix, apv.vendor_id, apv.product_id);
		ROS_INFO_NAMED("sys", "%s: UID:                 %016llx", prefix, (long long int)apv.uid);
	}

	void process_autopilot_version_apm_quirk(mavlink::common::msg::AUTOPILOT_VERSION &apv, uint8_t sysid, uint8_t compid)
	{
		char prefix[16];
		std::snprintf(prefix, sizeof(prefix), "VER: %d.%d", sysid, compid);

		// Note based on current APM's impl.
		// APM uses custom version array[8] as a string
		ROS_INFO_NAMED("sys", "%s: Capabilities         0x%016llx", prefix, (long long int)apv.capabilities);
		ROS_INFO_NAMED("sys", "%s: Flight software:     %08x (%*s)",
				prefix,
				apv.flight_sw_version,
				8, apv.flight_custom_version.data());
		ROS_INFO_NAMED("sys", "%s: Middleware software: %08x (%*s)",
				prefix,
				apv.middleware_sw_version,
				8, apv.middleware_custom_version.data());
		ROS_INFO_NAMED("sys", "%s: OS software:         %08x (%*s)",
				prefix,
				apv.os_sw_version,
				8, apv.os_custom_version.data());
		ROS_INFO_NAMED("sys", "%s: Board hardware:      %08x", prefix, apv.board_version);
		ROS_INFO_NAMED("sys", "%s: VID/PID:             %04x:%04x", prefix, apv.vendor_id, apv.product_id);
		ROS_INFO_NAMED("sys", "%s: UID:                 %016llx", prefix, (long long int)apv.uid);
	}

	void publish_disconnection() {
		auto state_msg = boost::make_shared<mavros_msgs::State>();
		state_msg->header.stamp = ros::Time::now();
		state_msg->connected = false;
		state_msg->armed = false;
		state_msg->guided = false;
		state_msg->mode = "";
		state_msg->system_status = enum_value(MAV_STATE::UNINIT);

		state_pub.publish(state_msg);
	}

	/* -*- message handlers -*- */

	void handle_heartbeat(const mavlink::mavlink_message_t *msg, mavlink::minimal::msg::HEARTBEAT &hb)
	{
		using mavlink::minimal::MAV_MODE_FLAG;

		// Store generic info of all heartbeats seen
		auto it = find_or_create_vehicle_info(msg->sysid, msg->compid);

		auto vehicle_mode = m_uas->str_mode_v10(hb.base_mode, hb.custom_mode);
		auto stamp = ros::Time::now();

		// Update vehicle data
		it->second.header.stamp = stamp;
		it->second.available_info |= mavros_msgs::VehicleInfo::HAVE_INFO_HEARTBEAT;
		it->second.autopilot = hb.autopilot;
		it->second.type = hb.type;
		it->second.system_status = hb.system_status;
		it->second.base_mode = hb.base_mode;
		it->second.custom_mode = hb.custom_mode;
		it->second.mode = vehicle_mode;

		if (!(hb.base_mode & enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED))) {
			it->second.mode_id = hb.base_mode;
		} else {
			it->second.mode_id = hb.custom_mode;
		}

		// Continue from here only if vehicle is my target
		if (!m_uas->is_my_target(msg->sysid, msg->compid)) {
			ROS_DEBUG_NAMED("sys", "HEARTBEAT from [%d, %d] dropped.", msg->sysid, msg->compid);
			return;
		}

		// update context && setup connection timeout
		m_uas->update_heartbeat(hb.type, hb.autopilot, hb.base_mode);
		m_uas->update_connection_status(true);
		timeout_timer.stop();
		timeout_timer.start();

		// build state message after updating uas
		auto state_msg = boost::make_shared<mavros_msgs::State>();
		state_msg->header.stamp = stamp;
		state_msg->connected = true;
		state_msg->armed = !!(hb.base_mode & enum_value(MAV_MODE_FLAG::SAFETY_ARMED));
		state_msg->guided = !!(hb.base_mode & enum_value(MAV_MODE_FLAG::GUIDED_ENABLED));
		state_msg->manual_input = !!(hb.base_mode & enum_value(MAV_MODE_FLAG::MANUAL_INPUT_ENABLED));
		state_msg->mode = vehicle_mode;
		state_msg->system_status = hb.system_status;

		state_pub.publish(state_msg);
		hb_diag.tick(hb.type, hb.autopilot, state_msg->mode, hb.system_status);
	}

	void handle_extended_sys_state(const mavlink::mavlink_message_t *msg, mavlink::common::msg::EXTENDED_SYS_STATE &state)
	{
		auto state_msg = boost::make_shared<mavros_msgs::ExtendedState>();
		state_msg->header.stamp = ros::Time::now();
		state_msg->vtol_state = state.vtol_state;
		state_msg->landed_state = state.landed_state;

		extended_state_pub.publish(state_msg);
	}

	void handle_sys_status(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SYS_STATUS &stat)
	{
		float volt = stat.voltage_battery / 1000.0f;	// mV
		float curr = stat.current_battery / 100.0f;	// 10 mA or -1
		float rem = stat.battery_remaining / 100.0f;	// or -1

		battery_voltage = volt;
		sys_diag.set(stat);
		batt_diag.set(volt, curr, rem);

		if (has_battery_status)
			return;

		auto batt_msg = boost::make_shared<BatteryMsg>();
		batt_msg->header.stamp = ros::Time::now();

#ifdef HAVE_SENSOR_MSGS_BATTERYSTATE_MSG
		batt_msg->voltage = volt;
		batt_msg->current = -curr;
		batt_msg->charge = NAN;
		batt_msg->capacity = NAN;
		batt_msg->design_capacity = NAN;
		batt_msg->percentage = rem;
		batt_msg->power_supply_status = BatteryMsg::POWER_SUPPLY_STATUS_DISCHARGING;
		batt_msg->power_supply_health = BatteryMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
		batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		batt_msg->present = true;
		batt_msg->cell_voltage.clear();	// not necessary. Cell count and Voltage unknown.
		batt_msg->location = "";
		batt_msg->serial_number = "";
#else	// mavros_msgs::BatteryStatus
		batt_msg->voltage = volt;
		batt_msg->current = curr;
		batt_msg->remaining = rem;
#endif

		batt_pub.publish(batt_msg);
	}

	void handle_statustext(const mavlink::mavlink_message_t *msg, mavlink::common::msg::STATUSTEXT &textm)
	{
		auto text = mavlink::to_string(textm.text);
		process_statustext_normal(textm.severity, text);

		auto st_msg = boost::make_shared<mavros_msgs::StatusText>();
		st_msg->header.stamp = ros::Time::now();
		st_msg->severity = textm.severity;
		st_msg->text = text;
		statustext_pub.publish(st_msg);
	}

	void handle_meminfo(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::MEMINFO &mem)
	{
		mem_diag.set(mem.freemem, mem.brkval);
	}

	void handle_hwstatus(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::HWSTATUS &hwst)
	{
		hwst_diag.set(hwst.Vcc, hwst.I2Cerr);
	}

	void handle_autopilot_version(const mavlink::mavlink_message_t *msg, mavlink::common::msg::AUTOPILOT_VERSION &apv)
	{
		// we want to store only FCU caps
		if (m_uas->is_my_target(msg->sysid, msg->compid)) {
			autopilot_version_timer.stop();
			m_uas->update_capabilities(true, apv.capabilities);
		}

		// but print all version responses
		if (m_uas->is_ardupilotmega())
			process_autopilot_version_apm_quirk(apv, msg->sysid, msg->compid);
		else
			process_autopilot_version_normal(apv, msg->sysid, msg->compid);

		// Store generic info of all autopilot seen
		auto it = find_or_create_vehicle_info(msg->sysid, msg->compid);

		// Update vehicle data
		it->second.header.stamp = ros::Time::now();
		it->second.available_info |= mavros_msgs::VehicleInfo::HAVE_INFO_AUTOPILOT_VERSION;
		it->second.capabilities = apv.capabilities;
		it->second.flight_sw_version = apv.flight_sw_version;
		it->second.middleware_sw_version = apv.middleware_sw_version;
		it->second.os_sw_version = apv.os_sw_version;
		it->second.board_version = apv.board_version;
		it->second.vendor_id = apv.vendor_id;
		it->second.product_id = apv.product_id;
		it->second.uid = apv.uid;
	}

	void handle_battery_status(const mavlink::mavlink_message_t *msg, mavlink::common::msg::BATTERY_STATUS &bs)
	{
		// PX4.
#ifdef HAVE_SENSOR_MSGS_BATTERYSTATE_MSG
		using BT = mavlink::common::MAV_BATTERY_TYPE;

		has_battery_status = true;

		auto batt_msg = boost::make_shared<BatteryMsg>();
		batt_msg->header.stamp = ros::Time::now();

		batt_msg->voltage = battery_voltage;
		batt_msg->current = -(bs.current_battery / 100.0f);	// 10 mA
		batt_msg->charge = NAN;
		batt_msg->capacity = NAN;
		batt_msg->design_capacity = NAN;
		batt_msg->percentage = bs.battery_remaining / 100.0f;
		batt_msg->power_supply_status = BatteryMsg::POWER_SUPPLY_STATUS_DISCHARGING;
		batt_msg->power_supply_health = BatteryMsg::POWER_SUPPLY_HEALTH_UNKNOWN;

		switch (bs.type) {
		// [[[cog:
		// for f in (
		//     'LIPO',
		//     'LIFE',
		//     'LION',
		//     'NIMH',
		//     'UNKNOWN'):
		//     cog.outl("case enum_value(BT::%s):" % f)
		//     if f == 'UNKNOWN':
		//         cog.outl("default:")
		//     cog.outl("\tbatt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_%s;" % f)
		//     cog.outl("\tbreak;")
		// ]]]
		case enum_value(BT::LIPO):
			batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_LIPO;
			break;
		case enum_value(BT::LIFE):
			batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_LIFE;
			break;
		case enum_value(BT::LION):
			batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		case enum_value(BT::NIMH):
			batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_NIMH;
			break;
		case enum_value(BT::UNKNOWN):
		default:
			batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
			break;
		// [[[end]]] (checksum: 2bf14a81b3027f14ba1dd9b4c086a41d)
		}

		batt_msg->present = true;

		batt_msg->cell_voltage.clear();
		batt_msg->cell_voltage.reserve(bs.voltages.size());
		for (auto v : bs.voltages) {
			if (v == UINT16_MAX)
				break;

			batt_msg->cell_voltage.push_back(v / 1000.0f);	// 1 mV
		}

		batt_msg->location = utils::format("id%u", bs.id);
		batt_msg->serial_number = "";

		batt_pub.publish(batt_msg);
#endif
	}

	void handle_estimator_status(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ESTIMATOR_STATUS &status)
	{
		using ESF = mavlink::common::ESTIMATOR_STATUS_FLAGS;
		
		auto est_status_msg = boost::make_shared<mavros_msgs::EstimatorStatus>();
		est_status_msg->header.stamp = ros::Time::now();

		// [[[cog:
		// import pymavlink.dialects.v20.common as common
		// ename = 'ESTIMATOR_STATUS_FLAGS'
		// ename_pfx2 = 'ESTIMATOR_'
		//
		// enum = sorted(common.enums[ename].items())
		// enum.pop() # -> remove ENUM_END
		//
		// for k, e in enum:
		//     desc = e.description.split(' ', 1)[1] if e.description.startswith('0x') else e.description
		//     esf = e.name
		//
		//     if esf.startswith(ename + '_'):
		//         esf = esf[len(ename) + 1:]
		//     if esf.startswith(ename_pfx2):
		//         esf = esf[len(ename_pfx2):]
		//     if esf[0].isdigit():
		//         esf = 'SENSOR_' + esf
		//     cog.outl("est_status_msg->%s_status_flag = !!(status.flags & enum_value(ESF::%s));" % (esf.lower(), esf))
		// ]]]
		est_status_msg->attitude_status_flag = !!(status.flags & enum_value(ESF::ATTITUDE));
		est_status_msg->velocity_horiz_status_flag = !!(status.flags & enum_value(ESF::VELOCITY_HORIZ));
		est_status_msg->velocity_vert_status_flag = !!(status.flags & enum_value(ESF::VELOCITY_VERT));
		est_status_msg->pos_horiz_rel_status_flag = !!(status.flags & enum_value(ESF::POS_HORIZ_REL));
		est_status_msg->pos_horiz_abs_status_flag = !!(status.flags & enum_value(ESF::POS_HORIZ_ABS));
		est_status_msg->pos_vert_abs_status_flag = !!(status.flags & enum_value(ESF::POS_VERT_ABS));
		est_status_msg->pos_vert_agl_status_flag = !!(status.flags & enum_value(ESF::POS_VERT_AGL));
		est_status_msg->const_pos_mode_status_flag = !!(status.flags & enum_value(ESF::CONST_POS_MODE));
		est_status_msg->pred_pos_horiz_rel_status_flag = !!(status.flags & enum_value(ESF::PRED_POS_HORIZ_REL));
		est_status_msg->pred_pos_horiz_abs_status_flag = !!(status.flags & enum_value(ESF::PRED_POS_HORIZ_ABS));
		est_status_msg->gps_glitch_status_flag = !!(status.flags & enum_value(ESF::GPS_GLITCH));
		est_status_msg->accel_error_status_flag = !!(status.flags & enum_value(ESF::ACCEL_ERROR));
		// [[[end]]] (checksum: 7828381ee4002ea6b61a8f528ae4d12d)

		estimator_status_pub.publish(est_status_msg);
	}

	/* -*- timer callbacks -*- */

	void timeout_cb(const ros::TimerEvent &event)
	{
		m_uas->update_connection_status(false);
	}

	void heartbeat_cb(const ros::TimerEvent &event)
	{
		using mavlink::common::MAV_MODE;

		mavlink::minimal::msg::HEARTBEAT hb {};

		hb.type = enum_value(conn_heartbeat_mav_type); //! @todo patch PX4 so it can also handle this type as datalink
		hb.autopilot = enum_value(MAV_AUTOPILOT::INVALID);
		hb.base_mode = enum_value(MAV_MODE::MANUAL_ARMED);
		hb.custom_mode = 0;
		hb.system_status = enum_value(MAV_STATE::ACTIVE);

		UAS_FCU(m_uas)->send_message_ignore_drop(hb);
	}

	void autopilot_version_cb(const ros::TimerEvent &event)
	{
		using mavlink::common::MAV_CMD;

		bool ret = false;

		// Request from all first 3 times, then fallback to unicast
		bool do_broadcast = version_retries > RETRIES_COUNT / 2;

		try {
			auto client = nh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

			mavros_msgs::CommandLong cmd{};

			cmd.request.broadcast = do_broadcast;
			cmd.request.command = enum_value(MAV_CMD::REQUEST_AUTOPILOT_CAPABILITIES);
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
			m_uas->update_capabilities(false);
			autopilot_version_timer.stop();
			ROS_WARN_NAMED("sys", "VER: your FCU don't support AUTOPILOT_VERSION, "
					"switched to default capabilities");
		}
	}

	void connection_cb(bool connected) override
	{
		has_battery_status = false;

		// if connection changes, start delayed version request
		version_retries = RETRIES_COUNT;
		if (connected)
			autopilot_version_timer.start();
		else
			autopilot_version_timer.stop();

		// add/remove APM diag tasks
		if (connected && disable_diag && m_uas->is_ardupilotmega()) {
			UAS_DIAG(m_uas).add(mem_diag);
			UAS_DIAG(m_uas).add(hwst_diag);
		}
		else {
			UAS_DIAG(m_uas).removeByName(mem_diag.getName());
			UAS_DIAG(m_uas).removeByName(hwst_diag.getName());
		}

		if (!connected) {
			// publish connection change
			publish_disconnection();

			// Clear known vehicles
			vehicles.clear();
		}
	}

	/* -*- subscription callbacks -*- */

	void statustext_cb(const mavros_msgs::StatusText::ConstPtr &req) {
		mavlink::common::msg::STATUSTEXT statustext {};
		statustext.severity = req->severity;

		// Limit the length of the string by null-terminating at the 50-th character
		ROS_WARN_COND_NAMED(req->text.length() >= statustext.text.size(), "sys",
				"Status text too long: truncating...");
		mavlink::set_string_z(statustext.text, req->text);

		UAS_FCU(m_uas)->send_message_ignore_drop(statustext);
	}

	/* -*- ros callbacks -*- */

	bool set_rate_cb(mavros_msgs::StreamRate::Request &req,
			mavros_msgs::StreamRate::Response &res)
	{
		mavlink::common::msg::REQUEST_DATA_STREAM rq;

		rq.target_system = m_uas->get_tgt_system();
		rq.target_component = m_uas->get_tgt_component();
		rq.req_stream_id = req.stream_id;
		rq.req_message_rate = req.message_rate;
		rq.start_stop = (req.on_off) ? 1 : 0;

		UAS_FCU(m_uas)->send_message_ignore_drop(rq);
		return true;
	}

	bool set_mode_cb(mavros_msgs::SetMode::Request &req,
			mavros_msgs::SetMode::Response &res)
	{
		using mavlink::minimal::MAV_MODE_FLAG;

		uint8_t base_mode = req.base_mode;
		uint32_t custom_mode = 0;

		if (req.custom_mode != "") {
			if (!m_uas->cmode_from_str(req.custom_mode, custom_mode)) {
				res.mode_sent = false;
				return true;
			}

			/**
			 * @note That call may trigger unexpected arming change because
			 *       base_mode arming flag state based on previous HEARTBEAT
			 *       message value.
			 */
			base_mode |= (m_uas->get_armed()) ? enum_value(MAV_MODE_FLAG::SAFETY_ARMED) : 0;
			base_mode |= (m_uas->get_hil_state()) ? enum_value(MAV_MODE_FLAG::HIL_ENABLED) : 0;
			base_mode |= enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED);
		}

		mavlink::common::msg::SET_MODE sm;
		sm.target_system = m_uas->get_tgt_system();
		sm.base_mode = base_mode;
		sm.custom_mode = custom_mode;

		UAS_FCU(m_uas)->send_message_ignore_drop(sm);
		res.mode_sent = true;
		return true;
	}

	bool vehicle_info_get_cb(mavros_msgs::VehicleInfoGet::Request &req,
			mavros_msgs::VehicleInfoGet::Response &res)
	{
		if (req.get_all) {
			// Send all vehicles
			for (const auto &got : vehicles) {
				res.vehicles.emplace_back(got.second);
			}

			res.success = true;
			return res.success;
		}

		uint8_t req_sysid = req.sysid;
		uint8_t req_compid = req.compid;

		if (req.sysid == 0 && req.compid == 0) {
			// use target
			req_sysid = m_uas->get_tgt_system();
			req_compid = m_uas->get_tgt_component();
		}

		uint16_t key = get_vehicle_key(req_sysid, req_compid);
		auto it = vehicles.find(key);

		if (it == vehicles.end()) {
			// Vehicle not found
			res.success = false;
			return res.success;
		}

		res.vehicles.emplace_back(it->second);
		res.success = true;
		return res.success;
	}

    bool set_message_interval_cb(mavros_msgs::MessageInterval::Request &req,
            mavros_msgs::MessageInterval::Response &res)
    {
        using mavlink::common::MAV_CMD;

        try {
            auto client = nh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

            // calculate interval
            float interval_us;
            if (req.message_rate < 0) {
                interval_us = -1.0f;
            } else if (req.message_rate == 0) {
                interval_us = 0.0f;
            } else {
                interval_us = 1000000.0f / req.message_rate;
            }

            mavros_msgs::CommandLong cmd{};

            cmd.request.broadcast = false;
            cmd.request.command = enum_value(MAV_CMD::SET_MESSAGE_INTERVAL);
            cmd.request.confirmation = false;
            cmd.request.param1 = req.message_id;
            cmd.request.param2 = interval_us;

            ROS_DEBUG_NAMED("sys", "SetMessageInterval: Request msgid %u at %f hz",
                    req.message_id, req.message_rate);
            res.success = client.call(cmd);
        }
        catch (ros::InvalidNameException &ex) {
            ROS_ERROR_NAMED("sys", "SetMessageInterval: %s", ex.what());
        }

        ROS_ERROR_COND_NAMED(!res.success, "sys", "SetMessageInterval: command plugin service call failed!");

        return res.success;
    }
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SystemStatusPlugin, mavros::plugin::PluginBase)

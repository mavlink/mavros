/**
 * @brief Mount Control plugin
 * @file mount_control.cpp
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Dr.-Ing. Amilcar do Carmo Lucas <amilcar.lucas@iav.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2019 Jaeyoung Lim.
 * Copyright 2021 Dr.-Ing. Amilcar do Carmo Lucas <amilcar.lucas@iav.de>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/MountControl.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/MountConfigure.h>

namespace mavros {
namespace extra_plugins {
//! Mavlink enumerations
using mavlink::common::MAV_MOUNT_MODE;
using mavlink::common::MAV_CMD;
using utils::enum_value;

/**
 * @brief Mount diagnostic updater
 */
class MountStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
	MountStatusDiag(const std::string &name) :
		diagnostic_updater::DiagnosticTask(name),
		_last_orientation_update(0, 0),
		_debounce_s(NAN),
		_roll_deg(NAN),
		_pitch_deg(NAN),
		_yaw_deg(NAN),
		_setpoint_roll_deg(NAN),
		_setpoint_pitch_deg(NAN),
		_setpoint_yaw_deg(NAN),
		_err_threshold_deg(NAN),
		_error_detected(false),
		_mode(255)
	{ }

	void set_err_threshold_deg(float threshold_deg) {
		std::lock_guard<std::mutex> lock(mutex);
		_err_threshold_deg = threshold_deg;
	}

	void set_debounce_s(double debounce_s) {
		std::lock_guard<std::mutex> lock(mutex);
		_debounce_s = debounce_s;
	}

	void set_status(float roll_deg, float pitch_deg, float yaw_deg, ros::Time timestamp) {
		std::lock_guard<std::mutex> lock(mutex);
		_roll_deg = roll_deg;
		_pitch_deg = pitch_deg;
		_yaw_deg = yaw_deg;
		_last_orientation_update = timestamp;
	}

	void set_setpoint(float roll_deg, float pitch_deg, float yaw_deg, uint8_t mode) {
		std::lock_guard<std::mutex> lock(mutex);
		_setpoint_roll_deg = roll_deg;
		_setpoint_pitch_deg = pitch_deg;
		_setpoint_yaw_deg = yaw_deg;
		_mode = mode;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		float roll_err_deg;
		float pitch_err_deg;
		float yaw_err_deg;
		bool error_detected = false;
		bool stale = false;

		if (_mode != mavros_msgs::MountControl::MAV_MOUNT_MODE_MAVLINK_TARGETING) {
			// Can only directly compare the MAV_CMD_DO_MOUNT_CONTROL angles with the MOUNT_ORIENTATION angles when in MAVLINK_TARGETING mode
			stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Can not diagnose in this targeting mode");
			stat.addf("Mode", "%d", _mode);
			return;
		}

		const ros::Time now = ros::Time::now();
		{
			std::lock_guard<std::mutex> lock(mutex);
			roll_err_deg = _setpoint_roll_deg - _roll_deg;
			pitch_err_deg = _setpoint_pitch_deg - _pitch_deg;
			yaw_err_deg = _setpoint_yaw_deg - _yaw_deg;

			// detect errors (setpoint != current angle)
			if (fabs(roll_err_deg) > _err_threshold_deg) {
				error_detected = true;
			}
			if (fabs(pitch_err_deg) > _err_threshold_deg) {
				error_detected = true;
			}
			if (fabs(yaw_err_deg) > _err_threshold_deg) {
				error_detected = true;
			}
			if (now - _last_orientation_update > ros::Duration(5, 0)) {
				stale = true;
			}
			// accessing the _debounce_s variable should be done inside this mutex,
			// but we can treat it as an atomic variable, and save the trouble
		}

		// detect error state changes
		if (!_error_detected && error_detected) {
			_error_started = now;
			_error_detected = true;
		}
		if (_error_detected && !error_detected) {
			_error_detected = false;
		}

		// debounce errors
		if (stale) {
			stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, "No MOUNT_ORIENTATION received in the last 5 s");
		} else if (_error_detected && (now - _error_started > ros::Duration(_debounce_s))) {
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "angle error too high");
		} else {
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Normal");
		}

		stat.addf("Roll err (deg)", "%.1f", roll_err_deg);
		stat.addf("Pitch err (deg)", "%.1f", pitch_err_deg);
		stat.addf("Yaw err (deg)", "%.1f", yaw_err_deg);
	}

private:
	std::mutex mutex;
	ros::Time _error_started;
	ros::Time _last_orientation_update;
	double _debounce_s;
	float _roll_deg;
	float _pitch_deg;
	float _yaw_deg;
	float _setpoint_roll_deg;
	float _setpoint_pitch_deg;
	float _setpoint_yaw_deg;
	float _err_threshold_deg;
	bool _error_detected;
	uint8_t _mode;
};

/**
 * @brief Mount Control plugin
 *
 * Publishes Mission commands to control the camera or antenna mount.
 * @see command_cb()
 */
class MountControlPlugin : public plugin::PluginBase {
public:
	MountControlPlugin() : PluginBase(),
		nh("~"),
		mount_nh("~mount_control"),
		mount_diag("Mount")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		command_sub = mount_nh.subscribe("command", 10, &MountControlPlugin::command_cb, this);
		mount_orientation_pub = mount_nh.advertise<geometry_msgs::Quaternion>("orientation", 10);
		mount_status_pub = mount_nh.advertise<geometry_msgs::Vector3Stamped>("status", 10);
		configure_srv = mount_nh.advertiseService("configure", &MountControlPlugin::mount_configure_cb, this);

		// some gimbals send negated/inverted angle measurements
		// these parameters correct that to obey the MAVLink frame convention
		mount_nh.param<bool>("negate_measured_roll", negate_measured_roll, false);
		mount_nh.param<bool>("negate_measured_pitch", negate_measured_pitch, false);
		mount_nh.param<bool>("negate_measured_yaw", negate_measured_yaw, false);
		if (!mount_nh.getParam("negate_measured_roll", negate_measured_roll)) {
			ROS_WARN("Could not retrive negate_measured_roll parameter value, using default (%d)", negate_measured_roll);
		}
		if (!mount_nh.getParam("negate_measured_pitch", negate_measured_pitch)) {
			ROS_WARN("Could not retrive negate_measured_pitch parameter value, using default (%d)", negate_measured_pitch);
		}
		if (!mount_nh.getParam("negate_measured_yaw", negate_measured_yaw)) {
			ROS_WARN("Could not retrive negate_measured_yaw parameter value, using default (%d)", negate_measured_yaw);
		}

		bool disable_diag;
		if (nh.getParam("sys/disable_diag", disable_diag) && !disable_diag) {
			double debounce_s;
			double err_threshold_deg;
			mount_nh.param("debounce_s", debounce_s, 4.0);
			mount_nh.param("err_threshold_deg", err_threshold_deg, 10.0);
			if (!mount_nh.getParam("debounce_s", debounce_s)) {
				ROS_WARN("Could not retrive debounce_s parameter value, using default (%f)", debounce_s);
			}
			if (!mount_nh.getParam("err_threshold_deg", err_threshold_deg)) {
				ROS_WARN("Could not retrive err_threshold_deg parameter value, using default (%f)", err_threshold_deg);
			}
			mount_diag.set_debounce_s(debounce_s);
			mount_diag.set_err_threshold_deg(err_threshold_deg);
			UAS_DIAG(m_uas).add(mount_diag);
		}
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&MountControlPlugin::handle_mount_orientation),
			make_handler(&MountControlPlugin::handle_mount_status)
		};
	}

private:
	ros::NodeHandle nh;
	ros::NodeHandle mount_nh;
	ros::Subscriber command_sub;
	ros::Publisher mount_orientation_pub;
	ros::Publisher mount_status_pub;
	ros::ServiceServer configure_srv;

	MountStatusDiag mount_diag;
	bool negate_measured_roll;
	bool negate_measured_pitch;
	bool negate_measured_yaw;

	/**
	 * @brief Publish the mount orientation
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#MOUNT_ORIENTATION
	 * @param msg   the mavlink message
	 * @param mo	received MountOrientation msg
	 */
	void handle_mount_orientation(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MOUNT_ORIENTATION &mo)
	{
		const auto timestamp = ros::Time::now();
		// some gimbals send negated/inverted angle measurements, correct that to obey the MAVLink frame convention
		if (negate_measured_roll) {
			mo.roll = -mo.roll;
		}
		if (negate_measured_pitch) {
			mo.pitch = -mo.pitch;
		}
		if (negate_measured_yaw) {
			mo.yaw = -mo.yaw;
			mo.yaw_absolute = -mo.yaw_absolute;
		}
		const auto q = ftf::quaternion_from_rpy(Eigen::Vector3d(mo.roll, mo.pitch, mo.yaw) * M_PI / 180.0);
		geometry_msgs::Quaternion quaternion_msg;
		tf::quaternionEigenToMsg(q, quaternion_msg);
		mount_orientation_pub.publish(quaternion_msg);

		mount_diag.set_status(mo.roll, mo.pitch, mo.yaw_absolute, timestamp);
	}

	/**
	 * @brief Publish the mount status
	 *
	 * @param msg   the mavlink message
	 * @param ms	received MountStatus msg
	 */
	void handle_mount_status(const mavlink::mavlink_message_t *, mavlink::ardupilotmega::msg::MOUNT_STATUS &ms)
	{
		geometry_msgs::Vector3Stamped publish_msg;

		publish_msg.header.stamp = ros::Time::now();

		publish_msg.header.frame_id = std::to_string(ms.target_component);

		auto vec = Eigen::Vector3d(ms.pointing_b, ms.pointing_a, ms.pointing_c) * M_PI / 18000.0;
		tf::vectorEigenToMsg(vec, publish_msg.vector);

		mount_status_pub.publish(publish_msg);

		// pointing_X is cdeg
		auto q = ftf::quaternion_from_rpy(Eigen::Vector3d(ms.pointing_b, ms.pointing_a, ms.pointing_c) * M_PI / 18000.0);
		geometry_msgs::Quaternion quaternion_msg;
		tf::quaternionEigenToMsg(q, quaternion_msg);
		mount_orientation_pub.publish(quaternion_msg);
	}

	/**
	 * @brief Send mount control commands to vehicle
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL
	 * @param req	received MountControl msg
	 */
	void command_cb(const mavros_msgs::MountControl::ConstPtr &req)
	{
		mavlink::common::msg::COMMAND_LONG cmd {};

		cmd.target_system = m_uas->get_tgt_system();
		cmd.target_component = m_uas->get_tgt_component();
		cmd.command = enum_value(MAV_CMD::DO_MOUNT_CONTROL);
		cmd.param1 = req->pitch;
		cmd.param2 = req->roll;
		cmd.param3 = req->yaw;
		cmd.param4 = req->altitude;	//
		cmd.param5 = req->latitude;	// latitude in degrees * 1E7
		cmd.param6 = req->longitude;	// longitude in degrees * 1E7
		cmd.param7 = req->mode;	// MAV_MOUNT_MODE

		UAS_FCU(m_uas)->send_message_ignore_drop(cmd);

		mount_diag.set_setpoint(req->roll*0.01f, req->pitch*0.01f, req->yaw*0.01f, req->mode);
	}

	bool mount_configure_cb(mavros_msgs::MountConfigure::Request &req,
		mavros_msgs::MountConfigure::Response &res)
	{
		using mavlink::common::MAV_CMD;

		try {
			auto client = nh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

			mavros_msgs::CommandLong cmd{};

			cmd.request.broadcast = false;
			cmd.request.command = enum_value(MAV_CMD::DO_MOUNT_CONFIGURE);
			cmd.request.confirmation = false;
			cmd.request.param1 = req.mode;
			cmd.request.param2 = req.stabilize_roll;
			cmd.request.param3 = req.stabilize_pitch;
			cmd.request.param4 = req.stabilize_yaw;
			cmd.request.param5 = req.roll_input;
			cmd.request.param6 = req.pitch_input;
			cmd.request.param7 = req.yaw_input;

			ROS_DEBUG_NAMED("mount", "MountConfigure: Request mode %u ", req.mode);
			client.call(cmd);
			res.success = cmd.response.success;
		}
		catch (ros::InvalidNameException &ex) {
			ROS_ERROR_NAMED("mount", "MountConfigure: %s", ex.what());
		}

		ROS_ERROR_COND_NAMED(!res.success, "mount", "MountConfigure: command plugin service call failed!");

		return res.success;
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MountControlPlugin, mavros::plugin::PluginBase)

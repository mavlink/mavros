/**
 * @brief IMU and attitude data parser plugin
 * @file imu.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013-2017 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <cmath>
#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Vector3.h>

namespace mavros {
namespace std_plugins {
//! Gauss to Tesla coeff
static constexpr double GAUSS_TO_TESLA = 1.0e-4;
//! millTesla to Tesla coeff
static constexpr double MILLIT_TO_TESLA = 1000.0;
//! millRad/Sec to Rad/Sec coeff
static constexpr double MILLIRS_TO_RADSEC = 1.0e-3;
//! millG to m/s**2 coeff
static constexpr double MILLIG_TO_MS2 = 9.80665 / 1000.0;
//! millm/s**2 to m/s**2 coeff
static constexpr double MILLIMS2_TO_MS2 = 1.0e-3;
//! millBar to Pascal coeff
static constexpr double MILLIBAR_TO_PASCAL = 1.0e2;
//! Radians to degrees
static constexpr double RAD_TO_DEG = 180.0 / M_PI;


//! @brief IMU and attitude data publication plugin
class IMUPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	IMUPlugin() : PluginBase(),
		imu_nh("~imu"),
		has_hr_imu(false),
		has_raw_imu(false),
		has_scaled_imu(false),
		has_att_quat(false),
		received_linear_accel(false),
		linear_accel_vec_flu(Eigen::Vector3d::Zero()),
		linear_accel_vec_frd(Eigen::Vector3d::Zero())
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		double linear_stdev, angular_stdev, orientation_stdev, mag_stdev;

		/**
		 * @warning A rotation from the aircraft-frame to the base_link frame is applied.
		 * Additionally, it is reported the orientation of the vehicle to describe the
		 * transformation from the ENU frame to the base_link frame (ENU <-> base_link).
		 * THIS ORIENTATION IS NOT THE SAME AS THAT REPORTED BY THE FCU (NED <-> aircraft).
		 */
		imu_nh.param<std::string>("frame_id", frame_id, "base_link");
		imu_nh.param("linear_acceleration_stdev", linear_stdev, 0.0003);		// check default by MPU6000 spec
		imu_nh.param("angular_velocity_stdev", angular_stdev, 0.02 * (M_PI / 180.0));	// check default by MPU6000 spec
		imu_nh.param("orientation_stdev", orientation_stdev, 1.0);
		imu_nh.param("magnetic_stdev", mag_stdev, 0.0);

		setup_covariance(linear_acceleration_cov, linear_stdev);
		setup_covariance(angular_velocity_cov, angular_stdev);
		setup_covariance(orientation_cov, orientation_stdev);
		setup_covariance(magnetic_cov, mag_stdev);
		setup_covariance(unk_orientation_cov, 0.0);

		imu_pub = imu_nh.advertise<sensor_msgs::Imu>("data", 10);
		magn_pub = imu_nh.advertise<sensor_msgs::MagneticField>("mag", 10);
		temp_imu_pub = imu_nh.advertise<sensor_msgs::Temperature>("temperature_imu", 10);
		temp_baro_pub = imu_nh.advertise<sensor_msgs::Temperature>("temperature_baro", 10);
		static_press_pub = imu_nh.advertise<sensor_msgs::FluidPressure>("static_pressure", 10);
		diff_press_pub = imu_nh.advertise<sensor_msgs::FluidPressure>("diff_pressure", 10);
		imu_raw_pub = imu_nh.advertise<sensor_msgs::Imu>("data_raw", 10);

		// Reset has_* flags on connection change
		enable_connection_cb();
	}

	Subscriptions get_subscriptions() override {
		return {
			       make_handler(&IMUPlugin::handle_attitude),
			       make_handler(&IMUPlugin::handle_attitude_quaternion),
			       make_handler(&IMUPlugin::handle_highres_imu),
			       make_handler(&IMUPlugin::handle_raw_imu),
			       make_handler(&IMUPlugin::handle_scaled_imu),
			       make_handler(&IMUPlugin::handle_scaled_pressure),
		};
	}

private:
	ros::NodeHandle imu_nh;
	std::string frame_id;

	ros::Publisher imu_pub;
	ros::Publisher imu_raw_pub;
	ros::Publisher magn_pub;
	ros::Publisher temp_imu_pub;
	ros::Publisher temp_baro_pub;
	ros::Publisher static_press_pub;
	ros::Publisher diff_press_pub;

	bool has_hr_imu;
	bool has_raw_imu;
	bool has_scaled_imu;
	bool has_att_quat;
	bool received_linear_accel;
	Eigen::Vector3d linear_accel_vec_flu;
	Eigen::Vector3d linear_accel_vec_frd;
	ftf::Covariance3d linear_acceleration_cov;
	ftf::Covariance3d angular_velocity_cov;
	ftf::Covariance3d orientation_cov;
	ftf::Covariance3d unk_orientation_cov;
	ftf::Covariance3d magnetic_cov;

	/* -*- helpers -*- */

	/**
	 * @brief Setup 3x3 covariance matrix
	 * @param cov		Covariance matrix
	 * @param stdev		Standard deviation
	 * @remarks		Diagonal computed from the stdev
	 */
	void setup_covariance(ftf::Covariance3d &cov, double stdev)
	{
		ftf::EigenMapCovariance3d c(cov.data());

		c.setZero();
		if (stdev) {
			double sr = stdev * stdev;
			c.diagonal() << sr, sr, sr;
		}
		else {
			c(0,0) = -1.0;
		}
	}

	/**
	 * @brief Fill and publish IMU data message.
	 * @param time_boot_ms     Message timestamp (not syncronized)
	 * @param orientation_enu  Orientation in the base_link ENU frame
	 * @param orientation_ned  Orientation in the aircraft NED frame
	 * @param gyro_flu         Angular velocity/rate in the base_link Forward-Left-Up frame
	 * @param gyro_frd         Angular velocity/rate in the aircraft Forward-Right-Down frame
	 */
	void publish_imu_data(uint32_t time_boot_ms, Eigen::Quaterniond &orientation_enu,
				Eigen::Quaterniond &orientation_ned, Eigen::Vector3d &gyro_flu, Eigen::Vector3d &gyro_frd)
	{
		auto imu_ned_msg = boost::make_shared<sensor_msgs::Imu>();
		auto imu_enu_msg = boost::make_shared<sensor_msgs::Imu>();

		// Fill message header
		imu_enu_msg->header = m_uas->synchronized_header(frame_id, time_boot_ms);
		imu_ned_msg->header = m_uas->synchronized_header("aircraft", time_boot_ms);

		// Convert from Eigen::Quaternond to geometry_msgs::Quaternion
		tf::quaternionEigenToMsg(orientation_enu, imu_enu_msg->orientation);
		tf::quaternionEigenToMsg(orientation_ned, imu_ned_msg->orientation);

		// Convert from Eigen::Vector3d to geometry_msgs::Vector3
		tf::vectorEigenToMsg(gyro_flu, imu_enu_msg->angular_velocity);
		tf::vectorEigenToMsg(gyro_frd, imu_ned_msg->angular_velocity);

		// Eigen::Vector3d from HIGHRES_IMU or RAW_IMU, to geometry_msgs::Vector3
		tf::vectorEigenToMsg(linear_accel_vec_flu, imu_enu_msg->linear_acceleration);
		tf::vectorEigenToMsg(linear_accel_vec_frd, imu_ned_msg->linear_acceleration);

		// Pass ENU msg covariances
		imu_enu_msg->orientation_covariance = orientation_cov;
		imu_enu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_enu_msg->linear_acceleration_covariance = linear_acceleration_cov;

		// Pass NED msg covariances
		imu_ned_msg->orientation_covariance = orientation_cov;
		imu_ned_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_ned_msg->linear_acceleration_covariance = linear_acceleration_cov;

		if (!received_linear_accel) {
			// Set element 0 of covariance matrix to -1 if no data received as per sensor_msgs/Imu defintion
			imu_enu_msg->linear_acceleration_covariance[0] = -1;
			imu_ned_msg->linear_acceleration_covariance[0] = -1;
		}

		/** Store attitude in base_link ENU
		 *  @snippet src/plugins/imu.cpp store_enu
		 */
		// [store_enu]
		m_uas->update_attitude_imu_enu(imu_enu_msg);
		// [store_enu]

		/** Store attitude in aircraft NED
		 *  @snippet src/plugins/imu.cpp store_ned
		 */
		// [store_enu]
		m_uas->update_attitude_imu_ned(imu_ned_msg);
		// [store_ned]

		/** Publish only base_link ENU message
		 *  @snippet src/plugins/imu.cpp pub_enu
		 */
		// [pub_enu]
		imu_pub.publish(imu_enu_msg);
		// [pub_enu]
	}

	/**
	 * @brief Fill and publish IMU data_raw message; store linear acceleration for IMU data
	 * @param header      Message frame_id and timestamp
	 * @param gyro_flu    Orientation in the base_link Forward-Left-Up frame
	 * @param accel_flu   Linear acceleration in the base_link Forward-Left-Up frame
	 * @param accel_frd   Linear acceleration in the aircraft Forward-Right-Down frame
	 */
	void publish_imu_data_raw(std_msgs::Header &header, Eigen::Vector3d &gyro_flu,
				Eigen::Vector3d &accel_flu, Eigen::Vector3d &accel_frd)
	{
		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();

		// Fill message header
		imu_msg->header = header;

		tf::vectorEigenToMsg(gyro_flu, imu_msg->angular_velocity);
		tf::vectorEigenToMsg(accel_flu, imu_msg->linear_acceleration);

		// Save readings
		linear_accel_vec_flu = accel_flu;
		linear_accel_vec_frd = accel_frd;
		received_linear_accel = true;

		imu_msg->orientation_covariance = unk_orientation_cov;
		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

		// Publish message [ENU frame]
		imu_raw_pub.publish(imu_msg);
	}

	/**
	 * @brief Publish magnetic field data
	 * @param header	Message frame_id and timestamp
	 * @param mag_field	Magnetic field in the base_link ENU frame
	 */
	void publish_mag(std_msgs::Header &header, Eigen::Vector3d &mag_field)
	{
		auto magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

		// Fill message header
		magn_msg->header = header;

		tf::vectorEigenToMsg(mag_field, magn_msg->magnetic_field);
		magn_msg->magnetic_field_covariance = magnetic_cov;

		// Publish message [ENU frame]
		magn_pub.publish(magn_msg);
	}

	/* -*- message handlers -*- */

	/**
	 * @brief Handle ATTITUDE MAVlink message.
	 * Message specification: https://mavlink.io/en/messages/common.html#ATTITUDE
	 * @param msg	Received Mavlink msg
	 * @param att	ATTITUDE msg
	 */
	void handle_attitude(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ATTITUDE &att)
	{
		if (has_att_quat)
			return;

		/** Orientation on the NED-aicraft frame:
		 *  @snippet src/plugins/imu.cpp ned_aircraft_orient1
		 */
		// [ned_aircraft_orient1]
		auto ned_aircraft_orientation = ftf::quaternion_from_rpy(att.roll, att.pitch, att.yaw);
		// [ned_aircraft_orient1]

		/** Angular velocity on the NED-aicraft frame:
		 *  @snippet src/plugins/imu.cpp ned_ang_vel1
		 */
		// [frd_ang_vel1]
		auto gyro_frd = Eigen::Vector3d(att.rollspeed, att.pitchspeed, att.yawspeed);
		// [frd_ang_vel1]

		/** The RPY describes the rotation: aircraft->NED.
		 *  It is required to change this to aircraft->base_link:
		 *  @snippet src/plugins/imu.cpp ned->baselink->enu
		 */
		// [ned->baselink->enu]
		auto enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(
					ftf::transform_orientation_ned_enu(ned_aircraft_orientation));
		// [ned->baselink->enu]

		/** The angular velocity expressed in the aircraft frame.
		 *  It is required to apply the static rotation to get it into the base_link frame:
		 *  @snippet src/plugins/imu.cpp rotate_gyro
		 */
		// [rotate_gyro]
		auto gyro_flu = ftf::transform_frame_aircraft_baselink(gyro_frd);
		// [rotate_gyro]

		publish_imu_data(att.time_boot_ms, enu_baselink_orientation, ned_aircraft_orientation, gyro_flu, gyro_frd);
	}

	/**
	 * @brief Handle ATTITUDE_QUATERNION MAVlink message.
	 * Message specification: https://mavlink.io/en/messages/common.html/#ATTITUDE_QUATERNION
	 * @param msg		Received Mavlink msg
	 * @param att_q		ATTITUDE_QUATERNION msg
	 */
	void handle_attitude_quaternion(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ATTITUDE_QUATERNION &att_q)
	{
		ROS_INFO_COND_NAMED(!has_att_quat, "imu", "IMU: Attitude quaternion IMU detected!");
		has_att_quat = true;

		/** Orientation on the NED-aicraft frame:
		 *  @snippet src/plugins/imu.cpp ned_aircraft_orient2
		 */
		// [ned_aircraft_orient2]
		auto ned_aircraft_orientation = Eigen::Quaterniond(att_q.q1, att_q.q2, att_q.q3, att_q.q4);
		// [ned_aircraft_orient2]

		/** Angular velocity on the NED-aicraft frame:
		 *  @snippet src/plugins/imu.cpp ned_ang_vel2
		 */
		// [frd_ang_vel2]
		auto gyro_frd = Eigen::Vector3d(att_q.rollspeed, att_q.pitchspeed, att_q.yawspeed);
		// [frd_ang_vel2]

		/** MAVLink quaternion exactly matches Eigen convention.
		 *  The RPY describes the rotation: aircraft->NED.
		 *  It is required to change this to aircraft->base_link:
		 *  @snippet src/plugins/imu.cpp ned->baselink->enu
		 */
		auto enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(
					ftf::transform_orientation_ned_enu(ned_aircraft_orientation));

		/** The angular velocity expressed in the aircraft frame.
		 *  It is required to apply the static rotation to get it into the base_link frame:
		 *  @snippet src/plugins/imu.cpp rotate_gyro
		 */
		auto gyro_flu = ftf::transform_frame_aircraft_baselink(gyro_frd);

		publish_imu_data(att_q.time_boot_ms, enu_baselink_orientation, ned_aircraft_orientation, gyro_flu, gyro_frd);
	}

	/**
	 * @brief Handle HIGHRES_IMU MAVlink message.
	 * Message specification: https://mavlink.io/en/messages/common.html/#HIGHRES_IMU
	 * @param msg		Received Mavlink msg
	 * @param imu_hr	HIGHRES_IMU msg
	 */
	void handle_highres_imu(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HIGHRES_IMU &imu_hr)
	{
		ROS_INFO_COND_NAMED(!has_hr_imu, "imu", "IMU: High resolution IMU detected!");
		has_hr_imu = true;

		auto header = m_uas->synchronized_header(frame_id, imu_hr.time_usec);
		/** @todo Make more paranoic check of HIGHRES_IMU.fields_updated
		 */

		/** Check if accelerometer + gyroscope data are available.
		 *  Data is expressed in aircraft frame it is required to rotate to the base_link frame:
		 *  @snippet src/plugins/imu.cpp accel_available
		 */
		// [accel_available]
		if (imu_hr.fields_updated & ((7 << 3) | (7 << 0))) {
			auto gyro_flu = ftf::transform_frame_aircraft_baselink(Eigen::Vector3d(imu_hr.xgyro, imu_hr.ygyro, imu_hr.zgyro));

			auto accel_frd = Eigen::Vector3d(imu_hr.xacc, imu_hr.yacc, imu_hr.zacc);
			auto accel_flu = ftf::transform_frame_aircraft_baselink(accel_frd);

			publish_imu_data_raw(header, gyro_flu, accel_flu, accel_frd);
		}
		// [accel_available]

		/** Check if magnetometer data is available:
		 *  @snippet src/plugins/imu.cpp mag_available
		 */
		// [mag_available]
		if (imu_hr.fields_updated & (7 << 6)) {
			auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
						Eigen::Vector3d(imu_hr.xmag, imu_hr.ymag, imu_hr.zmag) * GAUSS_TO_TESLA);

			publish_mag(header, mag_field);
		}
		// [mag_available]

		/** Check if static pressure sensor data is available:
		 *  @snippet src/plugins/imu.cpp static_pressure_available
		 */
		// [static_pressure_available]
		if (imu_hr.fields_updated & (1 << 9)) {
			auto static_pressure_msg = boost::make_shared<sensor_msgs::FluidPressure>();

			static_pressure_msg->header = header;
			static_pressure_msg->fluid_pressure = imu_hr.abs_pressure;

			static_press_pub.publish(static_pressure_msg);
		}
		// [static_pressure_available]

		/** Check if differential pressure sensor data is available:
		 *  @snippet src/plugins/imu.cpp differential_pressure_available
		 */
		// [differential_pressure_available]
		if (imu_hr.fields_updated & (1 << 10)) {
			auto differential_pressure_msg = boost::make_shared<sensor_msgs::FluidPressure>();

			differential_pressure_msg->header = header;
			differential_pressure_msg->fluid_pressure = imu_hr.diff_pressure;

			diff_press_pub.publish(differential_pressure_msg);
		}
		// [differential_pressure_available]

		/** Check if temperature data is available:
		 *  @snippet src/plugins/imu.cpp temperature_available
		 */
		// [temperature_available]
		if (imu_hr.fields_updated & (1 << 12)) {
			auto temp_msg = boost::make_shared<sensor_msgs::Temperature>();

			temp_msg->header = header;
			temp_msg->temperature = imu_hr.temperature;

			temp_imu_pub.publish(temp_msg);
		}
		// [temperature_available]
	}

	/**
	 * @brief Handle RAW_IMU MAVlink message.
	 * Message specification: https://mavlink.io/en/messages/common.html/#RAW_IMU
	 * @param msg		Received Mavlink msg
	 * @param imu_raw	RAW_IMU msg
	 */
	void handle_raw_imu(const mavlink::mavlink_message_t *msg, mavlink::common::msg::RAW_IMU &imu_raw)
	{
		ROS_INFO_COND_NAMED(!has_raw_imu, "imu", "IMU: Raw IMU message used.");
		has_raw_imu = true;

		if (has_hr_imu || has_scaled_imu)
			return;

		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
		auto header = m_uas->synchronized_header(frame_id, imu_raw.time_usec);

		/** @note APM send SCALED_IMU data as RAW_IMU
		 */
		auto gyro_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
					Eigen::Vector3d(imu_raw.xgyro, imu_raw.ygyro, imu_raw.zgyro) * MILLIRS_TO_RADSEC);
		auto accel_frd = Eigen::Vector3d(imu_raw.xacc, imu_raw.yacc, imu_raw.zacc);
		auto accel_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(accel_frd);

		if (m_uas->is_ardupilotmega()) {
			accel_frd *= MILLIG_TO_MS2;
			accel_flu *= MILLIG_TO_MS2;
		} else if (m_uas->is_px4()) {
			accel_frd *= MILLIMS2_TO_MS2;
			accel_flu *= MILLIMS2_TO_MS2;
		}

		publish_imu_data_raw(header, gyro_flu, accel_flu, accel_frd);

		if (!m_uas->is_ardupilotmega()) {
			ROS_WARN_THROTTLE_NAMED(60, "imu", "IMU: linear acceleration on RAW_IMU known on APM only.");
			ROS_WARN_THROTTLE_NAMED(60, "imu", "IMU: ~imu/data_raw stores unscaled raw acceleration report.");
			linear_accel_vec_flu.setZero();
			linear_accel_vec_frd.setZero();
		}

		/** Magnetic field data:
		 *  @snippet src/plugins/imu.cpp mag_field
		 */
		// [mag_field]
		auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
					Eigen::Vector3d(imu_raw.xmag, imu_raw.ymag, imu_raw.zmag) * MILLIT_TO_TESLA);
		// [mag_field]

		publish_mag(header, mag_field);
	}

	/**
	 * @brief Handle SCALED_IMU MAVlink message.
	 * Message specification: https://mavlink.io/en/messages/common.html/#SCALED_IMU
	 * @param msg		Received Mavlink msg
	 * @param imu_raw	SCALED_IMU msg
	 */
	void handle_scaled_imu(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SCALED_IMU &imu_raw)
	{
		if (has_hr_imu)
			return;

		ROS_INFO_COND_NAMED(!has_scaled_imu, "imu", "IMU: Scaled IMU message used.");
		has_scaled_imu = true;

		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
		auto header = m_uas->synchronized_header(frame_id, imu_raw.time_boot_ms);

		auto gyro_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
					Eigen::Vector3d(imu_raw.xgyro, imu_raw.ygyro, imu_raw.zgyro) * MILLIRS_TO_RADSEC);
		auto accel_frd = Eigen::Vector3d(Eigen::Vector3d(imu_raw.xacc, imu_raw.yacc, imu_raw.zacc) * MILLIG_TO_MS2);
		auto accel_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(accel_frd);

		publish_imu_data_raw(header, gyro_flu, accel_flu, accel_frd);

		/** Magnetic field data:
		 *  @snippet src/plugins/imu.cpp mag_field
		 */
		auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
					Eigen::Vector3d(imu_raw.xmag, imu_raw.ymag, imu_raw.zmag) * MILLIT_TO_TESLA);

		publish_mag(header, mag_field);
	}

	/**
	 * @brief Handle SCALED_PRESSURE MAVlink message.
	 * Message specification: https://mavlink.io/en/messages/common.html/#SCALED_PRESSURE
	 * @param msg		Received Mavlink msg
	 * @param press		SCALED_PRESSURE msg
	 */
	void handle_scaled_pressure(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SCALED_PRESSURE &press)
	{
		if (has_hr_imu)
			return;

		auto header = m_uas->synchronized_header(frame_id, press.time_boot_ms);

		auto temp_msg = boost::make_shared<sensor_msgs::Temperature>();
		temp_msg->header = header;
		temp_msg->temperature = press.temperature / 100.0;
		temp_baro_pub.publish(temp_msg);

		auto static_pressure_msg = boost::make_shared<sensor_msgs::FluidPressure>();
		static_pressure_msg->header = header;
		static_pressure_msg->fluid_pressure = press.press_abs * 100.0;
		static_press_pub.publish(static_pressure_msg);

		auto differential_pressure_msg = boost::make_shared<sensor_msgs::FluidPressure>();
		differential_pressure_msg->header = header;
		differential_pressure_msg->fluid_pressure = press.press_diff * 100.0;
		diff_press_pub.publish(differential_pressure_msg);
	}

	// Checks for connection and overrides variable values
	void connection_cb(bool connected) override
	{
		has_hr_imu = false;
		has_scaled_imu = false;
		has_att_quat = false;
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::IMUPlugin, mavros::plugin::PluginBase)

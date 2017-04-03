/**
 * @brief IMU publish plugin
 * @file imu_pub.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013,2015,2016 Vladimir Ermakov.
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
/* Note: this coefficents before been inside plugin class,
 * but after #320 something broken and in resulting plugins.so
 * there no symbols for that constants.
 * That cause plugin loader failure.
 *
 * objdump -x plugins.so | grep MILLI
 */

//! Gauss to Tesla coeff
static constexpr double GAUSS_TO_TESLA = 1.0e-4;
//! millTesla to Tesla coeff
static constexpr double MILLIT_TO_TESLA = 1000.0;
//! millRad/Sec to Rad/Sec coeff
static constexpr double MILLIRS_TO_RADSEC = 1.0e-3;
//! millG to m/s**2 coeff
static constexpr double MILLIG_TO_MS2 = 9.80665 / 1000.0;
//! millBar to Pascal coeff
static constexpr double MILLIBAR_TO_PASCAL = 1.0e2;

static constexpr double RAD_TO_DEG = 180.0 / M_PI;


/**
 * @brief IMU data publication plugin
 */
class IMUPubPlugin : public plugin::PluginBase {
public:
	IMUPubPlugin() : PluginBase(),
		imu_nh("~imu"),
		has_hr_imu(false),
		has_scaled_imu(false),
		has_att_quat(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		double linear_stdev, angular_stdev, orientation_stdev, mag_stdev;

		// we rotate the data from the aircraft-frame to the base_link frame.
		// Additionally we report the orientation of the vehicle to describe the
		// transformation from the ENU frame to the base_link frame (ENU <-> base_link).
		// THIS ORIENTATION IS NOT THE SAME AS THAT REPORTED BY THE FCU (NED <-> aircraft)
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
		temp_pub = imu_nh.advertise<sensor_msgs::Temperature>("temperature", 10);
		press_pub = imu_nh.advertise<sensor_msgs::FluidPressure>("atm_pressure", 10);
		imu_raw_pub = imu_nh.advertise<sensor_msgs::Imu>("data_raw", 10);

		// reset has_* flags on connection change
		enable_connection_cb();
	}

	Subscriptions get_subscriptions() {
		return {
		       make_handler(&IMUPubPlugin::handle_attitude),
		       make_handler(&IMUPubPlugin::handle_attitude_quaternion),
		       make_handler(&IMUPubPlugin::handle_highres_imu),
		       make_handler(&IMUPubPlugin::handle_raw_imu),
		       make_handler(&IMUPubPlugin::handle_scaled_imu),
		       make_handler(&IMUPubPlugin::handle_scaled_pressure),
		};
	}

private:
	ros::NodeHandle imu_nh;
	std::string frame_id;

	ros::Publisher imu_pub;
	ros::Publisher imu_raw_pub;
	ros::Publisher magn_pub;
	ros::Publisher temp_pub;
	ros::Publisher press_pub;

	bool has_hr_imu;
	bool has_scaled_imu;
	bool has_att_quat;
	Eigen::Vector3d linear_accel_vec;
	ftf::Covariance3d linear_acceleration_cov;
	ftf::Covariance3d angular_velocity_cov;
	ftf::Covariance3d orientation_cov;
	ftf::Covariance3d unk_orientation_cov;
	ftf::Covariance3d magnetic_cov;

	/* -*- helpers -*- */

	void setup_covariance(ftf::Covariance3d &cov, double stdev)
	{
		std::fill(cov.begin(), cov.end(), 0.0);
		if (stdev == 0.0)
			cov[0] = -1.0;
		else {
			cov[0 + 0] = cov[3 + 1] = cov[6 + 2] = std::pow(stdev, 2);
		}
	}

	//! fill and publish imu/data message
	void publish_imu_data(
			uint32_t time_boot_ms,
			Eigen::Quaterniond &orientation,
			Eigen::Vector3d &gyro)
	{
		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();

		// fill
		imu_msg->header = m_uas->synchronized_header(frame_id, time_boot_ms);

		tf::quaternionEigenToMsg(orientation, imu_msg->orientation);
		tf::vectorEigenToMsg(gyro, imu_msg->angular_velocity);

		// vector from HIGHRES_IMU or RAW_IMU
		tf::vectorEigenToMsg(linear_accel_vec, imu_msg->linear_acceleration);

		imu_msg->orientation_covariance = orientation_cov;
		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

		// publish
		m_uas->update_attitude_imu(imu_msg);
		imu_pub.publish(imu_msg);
	}

	//! fill and publish imu/data_raw message, store linear acceleration for imu/data
	void publish_imu_data_raw(
			std_msgs::Header &header,
			Eigen::Vector3d &gyro,
			Eigen::Vector3d &accel)
	{
		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();

		// fill
		imu_msg->header = header;

		tf::vectorEigenToMsg(gyro, imu_msg->angular_velocity);
		tf::vectorEigenToMsg(accel, imu_msg->linear_acceleration);

		// save readings
		linear_accel_vec = accel;

		imu_msg->orientation_covariance = unk_orientation_cov;
		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

		// publish
		imu_raw_pub.publish(imu_msg);
	}

	void publish_mag(std_msgs::Header &header,
			Eigen::Vector3d &mag_field)
	{
		auto magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

		magn_msg->header = header;
		tf::vectorEigenToMsg(mag_field, magn_msg->magnetic_field);
		magn_msg->magnetic_field_covariance = magnetic_cov;

		magn_pub.publish(magn_msg);
	}

	/* -*- message handlers -*- */

	void handle_attitude(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ATTITUDE &att)
	{
		if (has_att_quat)
			return;

		//Here we have rpy describing the rotation: aircraft->NED
		//We need to change this to aircraft->ENU
		//And finally change it to baselink->ENU
		auto enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(
				ftf::transform_orientation_ned_enu(
					ftf::quaternion_from_rpy(att.roll, att.pitch, att.yaw)));

		//Here we have the angular velocity expressed in the aircraft frame
		//We need to apply the static rotation to get it into the base_link frame
		auto gyro = ftf::transform_frame_aircraft_baselink(
				Eigen::Vector3d(att.rollspeed, att.pitchspeed, att.yawspeed));

		publish_imu_data(att.time_boot_ms, enu_baselink_orientation, gyro);
	}

	// almost the same as handle_attitude(), but for ATTITUDE_QUATERNION
	void handle_attitude_quaternion(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ATTITUDE_QUATERNION &att_q)
	{
		ROS_INFO_COND_NAMED(!has_att_quat, "imu", "IMU: Attitude quaternion IMU detected!");
		has_att_quat = true;

		//MAVLink quaternion exactly matches Eigen convention
		//Here we have rpy describing the rotation: aircraft->NED
		//We need to change this to aircraft->ENU
		//And finally change it to baselink->ENU
		auto enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(
				ftf::transform_orientation_ned_enu(
					Eigen::Quaterniond(att_q.q1, att_q.q2, att_q.q3, att_q.q4)));

		//Here we have the angular velocity expressed in the aircraft frame
		//We need to apply the static rotation to get it into the base_link frame
		auto gyro = ftf::transform_frame_aircraft_baselink(
				Eigen::Vector3d(att_q.rollspeed, att_q.pitchspeed, att_q.yawspeed));

		publish_imu_data(att_q.time_boot_ms, enu_baselink_orientation, gyro);
	}

	void handle_highres_imu(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HIGHRES_IMU &imu_hr)
	{
		ROS_INFO_COND_NAMED(!has_hr_imu, "imu", "IMU: High resolution IMU detected!");
		has_hr_imu = true;

		auto header = m_uas->synchronized_header(frame_id, imu_hr.time_usec);
		// TODO make more paranoic check of HIGHRES_IMU.fields_updated

		// accelerometer + gyroscope data available
		// Data is expressed in aircraft frame we need to rotate to base_link frame
		if (imu_hr.fields_updated & ((7 << 3) | (7 << 0))) {
			auto gyro = ftf::transform_frame_aircraft_baselink(Eigen::Vector3d(imu_hr.xgyro, imu_hr.ygyro, imu_hr.zgyro));
			auto accel = ftf::transform_frame_aircraft_baselink(Eigen::Vector3d(imu_hr.xacc, imu_hr.yacc, imu_hr.zacc));

			publish_imu_data_raw(header, gyro, accel);
		}

		// magnetometer data available
		if (imu_hr.fields_updated & (7 << 6)) {
			auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
					Eigen::Vector3d(imu_hr.xmag, imu_hr.ymag, imu_hr.zmag) * GAUSS_TO_TESLA);

			publish_mag(header, mag_field);
		}

		// pressure data available
		if (imu_hr.fields_updated & (1 << 9)) {
			auto atmp_msg = boost::make_shared<sensor_msgs::FluidPressure>();

			atmp_msg->header = header;
			atmp_msg->fluid_pressure = imu_hr.abs_pressure * MILLIBAR_TO_PASCAL;

			press_pub.publish(atmp_msg);
		}

		if (imu_hr.fields_updated & (1 << 12)) {
			auto temp_msg = boost::make_shared<sensor_msgs::Temperature>();

			temp_msg->header = header;
			temp_msg->temperature = imu_hr.temperature;

			temp_pub.publish(temp_msg);
		}
	}

	void handle_raw_imu(const mavlink::mavlink_message_t *msg, mavlink::common::msg::RAW_IMU &imu_raw)
	{
		if (has_hr_imu || has_scaled_imu)
			return;

		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
		auto header = m_uas->synchronized_header(frame_id, imu_raw.time_usec);

		//! @note APM send SCALED_IMU data as RAW_IMU
		auto gyro = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xgyro, imu_raw.ygyro, imu_raw.zgyro) * MILLIRS_TO_RADSEC);
		auto accel = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xacc, imu_raw.yacc, imu_raw.zacc));

		if (m_uas->is_ardupilotmega())
			accel *= MILLIG_TO_MS2;

		publish_imu_data_raw(header, gyro, accel);

		if (!m_uas->is_ardupilotmega()) {
			ROS_WARN_THROTTLE_NAMED(60, "imu", "IMU: linear acceleration on RAW_IMU known on APM only.");
			ROS_WARN_THROTTLE_NAMED(60, "imu", "IMU: ~imu/data_raw stores unscaled raw acceleration report.");
			linear_accel_vec.setZero();
		}

		/* -*- magnetic vector -*- */
		auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xmag, imu_raw.ymag, imu_raw.zmag) * MILLIT_TO_TESLA);

		publish_mag(header, mag_field);
	}

	void handle_scaled_imu(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SCALED_IMU &imu_raw)
	{
		if (has_hr_imu)
			return;

		ROS_INFO_COND_NAMED(!has_scaled_imu, "imu", "IMU: Scaled IMU message used.");
		has_scaled_imu = true;

		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
		auto header = m_uas->synchronized_header(frame_id, imu_raw.time_boot_ms);

		auto gyro = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xgyro, imu_raw.ygyro, imu_raw.zgyro) * MILLIRS_TO_RADSEC);
		auto accel = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xacc, imu_raw.yacc, imu_raw.zacc) * MILLIG_TO_MS2);

		publish_imu_data_raw(header, gyro, accel);

		/* -*- magnetic vector -*- */
		auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xmag, imu_raw.ymag, imu_raw.zmag) * MILLIT_TO_TESLA);

		publish_mag(header, mag_field);
	}

	void handle_scaled_pressure(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SCALED_PRESSURE &press)
	{
		if (has_hr_imu)
			return;

		auto header = m_uas->synchronized_header(frame_id, press.time_boot_ms);

		auto temp_msg = boost::make_shared<sensor_msgs::Temperature>();
		temp_msg->header = header;
		temp_msg->temperature = press.temperature / 100.0;
		temp_pub.publish(temp_msg);

		auto atmp_msg = boost::make_shared<sensor_msgs::FluidPressure>();
		atmp_msg->header = header;
		atmp_msg->fluid_pressure = press.press_abs * 100.0;
		press_pub.publish(atmp_msg);
	}

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
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::IMUPubPlugin, mavros::plugin::PluginBase)


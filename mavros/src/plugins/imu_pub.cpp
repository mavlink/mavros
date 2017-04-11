/**
 * @brief IMU publish plugin
 * @file imu_pub.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <cmath>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Vector3.h>

namespace mavplugin {
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
class IMUPubPlugin : public MavRosPlugin {
public:
	IMUPubPlugin() :
		imu_nh("~imu"),
		uas(nullptr),
		has_hr_imu(false),
		has_scaled_imu(false),
		has_att_quat(false)
	{ };

	void initialize(UAS &uas_)
	{
		double linear_stdev, angular_stdev, orientation_stdev, mag_stdev;

		uas = &uas_;

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
		uas->sig_connection_changed.connect(boost::bind(&IMUPubPlugin::connection_cb, this, _1));
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE, &IMUPubPlugin::handle_attitude),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, &IMUPubPlugin::handle_attitude_quaternion),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_HIGHRES_IMU, &IMUPubPlugin::handle_highres_imu),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_RAW_IMU, &IMUPubPlugin::handle_raw_imu),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_SCALED_IMU, &IMUPubPlugin::handle_scaled_imu),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_SCALED_PRESSURE, &IMUPubPlugin::handle_scaled_pressure),
		};
	}

private:
	ros::NodeHandle imu_nh;
	UAS *uas;
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
	UAS::Covariance3d linear_acceleration_cov;
	UAS::Covariance3d angular_velocity_cov;
	UAS::Covariance3d orientation_cov;
	UAS::Covariance3d unk_orientation_cov;
	UAS::Covariance3d magnetic_cov;

	/* -*- helpers -*- */

	void setup_covariance(UAS::Covariance3d &cov, double stdev) {
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
		imu_msg->header = uas->synchronized_header(frame_id, time_boot_ms);

		tf::quaternionEigenToMsg(orientation, imu_msg->orientation);
		tf::vectorEigenToMsg(gyro, imu_msg->angular_velocity);

		// vector from HIGHRES_IMU or RAW_IMU
		tf::vectorEigenToMsg(linear_accel_vec, imu_msg->linear_acceleration);

		imu_msg->orientation_covariance = orientation_cov;
		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

		// publish
		uas->update_attitude_imu(imu_msg);
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

	void handle_attitude(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (has_att_quat)
			return;

		mavlink_attitude_t att;
		mavlink_msg_attitude_decode(msg, &att);

		//Here we have rpy describing the rotation: aircraft->NED
		//We need to change this to aircraft->ENU
		//And finally change it to baselink->ENU
		auto enu_baselink_orientation = UAS::transform_orientation_aircraft_baselink(
				UAS::transform_orientation_ned_enu(
					UAS::quaternion_from_rpy(att.roll, att.pitch, att.yaw)));

		//Here we have the angular velocity expressed in the aircraft frame
		//We need to apply the static rotation to get it into the base_link frame
		auto gyro = UAS::transform_frame_aircraft_baselink(
				Eigen::Vector3d(att.rollspeed, att.pitchspeed, att.yawspeed));

		publish_imu_data(att.time_boot_ms, enu_baselink_orientation, gyro);
	}

	// almost the same as handle_attitude(), but for ATTITUDE_QUATERNION
	void handle_attitude_quaternion(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_attitude_quaternion_t att_q;
		mavlink_msg_attitude_quaternion_decode(msg, &att_q);

		ROS_INFO_COND_NAMED(!has_att_quat, "imu", "IMU: Attitude quaternion IMU detected!");
		has_att_quat = true;

		//MAVLink quaternion exactly matches Eigen convention
		//Here we have rpy describing the rotation: aircraft->NED
		//We need to change this to aircraft->ENU
		//And finally change it to baselink->ENU
		auto enu_baselink_orientation = UAS::transform_orientation_aircraft_baselink(
				UAS::transform_orientation_ned_enu(
					Eigen::Quaterniond(att_q.q1, att_q.q2, att_q.q3, att_q.q4)));
		//Here we have the angular velocity expressed in the aircraft frame
		//We need to apply the static rotation to get it into the base_link frame
		auto gyro = UAS::transform_frame_aircraft_baselink(
				Eigen::Vector3d(att_q.rollspeed, att_q.pitchspeed, att_q.yawspeed));

		publish_imu_data(att_q.time_boot_ms, enu_baselink_orientation, gyro);
	}

	void handle_highres_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_highres_imu_t imu_hr;
		mavlink_msg_highres_imu_decode(msg, &imu_hr);

		ROS_INFO_COND_NAMED(!has_hr_imu, "imu", "IMU: High resolution IMU detected!");
		has_hr_imu = true;

		auto header = uas->synchronized_header(frame_id, imu_hr.time_usec);
		//! @todo make more paranoic check of HIGHRES_IMU.fields_updated

		// accelerometer + gyroscope data available
		// Data is expressed in aircraft frame we need to rotate to base_link frame
		if (imu_hr.fields_updated & ((7 << 3) | (7 << 0))) {
			auto gyro = UAS::transform_frame_aircraft_baselink(Eigen::Vector3d(imu_hr.xgyro, imu_hr.ygyro, imu_hr.zgyro));
			auto accel = UAS::transform_frame_aircraft_baselink(Eigen::Vector3d(imu_hr.xacc, imu_hr.yacc, imu_hr.zacc));

			publish_imu_data_raw(header, gyro, accel);
		}

		// magnetometer data available
		if (imu_hr.fields_updated & (7 << 6)) {
			auto mag_field = UAS::transform_frame_aircraft_baselink<Eigen::Vector3d>(
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

	void handle_raw_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (has_hr_imu || has_scaled_imu)
			return;

		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
		mavlink_raw_imu_t imu_raw;
		mavlink_msg_raw_imu_decode(msg, &imu_raw);

		auto header = uas->synchronized_header(frame_id, imu_raw.time_usec);

		//! @note APM send SCALED_IMU data as RAW_IMU
		auto gyro = UAS::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xgyro, imu_raw.ygyro, imu_raw.zgyro) * MILLIRS_TO_RADSEC);
		auto accel = UAS::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xacc, imu_raw.yacc, imu_raw.zacc));

		if (uas->is_ardupilotmega())
			accel *= MILLIG_TO_MS2;

		publish_imu_data_raw(header, gyro, accel);

		if (!uas->is_ardupilotmega()) {
			ROS_WARN_THROTTLE_NAMED(60, "imu", "IMU: linear acceleration on RAW_IMU known on APM only.");
			ROS_WARN_THROTTLE_NAMED(60, "imu", "IMU: ~imu/data_raw stores unscaled raw acceleration report.");
			linear_accel_vec.setZero();
		}

		/* -*- magnetic vector -*- */
		auto mag_field = UAS::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xmag, imu_raw.ymag, imu_raw.zmag) * MILLIT_TO_TESLA);

		publish_mag(header, mag_field);
	}

	void handle_scaled_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (has_hr_imu)
			return;

		ROS_INFO_COND_NAMED(!has_scaled_imu, "imu", "IMU: Scaled IMU message used.");
		has_scaled_imu = true;

		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
		mavlink_scaled_imu_t imu_raw;
		mavlink_msg_scaled_imu_decode(msg, &imu_raw);

		auto header = uas->synchronized_header(frame_id, imu_raw.time_boot_ms);

		auto gyro = UAS::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xgyro, imu_raw.ygyro, imu_raw.zgyro) * MILLIRS_TO_RADSEC);
		auto accel = UAS::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xacc, imu_raw.yacc, imu_raw.zacc) * MILLIG_TO_MS2);

		publish_imu_data_raw(header, gyro, accel);

		/* -*- magnetic vector -*- */
		auto mag_field = UAS::transform_frame_aircraft_baselink<Eigen::Vector3d>(
				Eigen::Vector3d(imu_raw.xmag, imu_raw.ymag, imu_raw.zmag) * MILLIT_TO_TESLA);

		publish_mag(header, mag_field);
	}

	void handle_scaled_pressure(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (has_hr_imu)
			return;

		mavlink_scaled_pressure_t press;
		mavlink_msg_scaled_pressure_decode(msg, &press);

		auto header = uas->synchronized_header(frame_id, press.time_boot_ms);

		auto temp_msg = boost::make_shared<sensor_msgs::Temperature>();
		temp_msg->header = header;
		temp_msg->temperature = press.temperature / 100.0;
		temp_pub.publish(temp_msg);

		auto atmp_msg = boost::make_shared<sensor_msgs::FluidPressure>();
		atmp_msg->header = header;
		atmp_msg->fluid_pressure = press.press_abs * 100.0;
		press_pub.publish(atmp_msg);
	}

	void connection_cb(bool connected) {
		has_hr_imu = false;
		has_scaled_imu = false;
		has_att_quat = false;
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::IMUPubPlugin, mavplugin::MavRosPlugin)


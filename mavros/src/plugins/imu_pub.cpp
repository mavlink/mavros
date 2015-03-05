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
#include <tf/transform_datatypes.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Vector3.h>

namespace mavplugin {
/**
 * @brief IMU data publication plugin
 */
class IMUPubPlugin : public MavRosPlugin {
public:
	IMUPubPlugin() :
		imu_nh("~imu"),
		uas(nullptr),
		linear_accel_vec(),
		has_hr_imu(false),
		has_scaled_imu(false),
		has_att_quat(false)
	{ };

	void initialize(UAS &uas_)
	{
		double linear_stdev, angular_stdev, orientation_stdev, mag_stdev;

		uas = &uas_;

		imu_nh.param<std::string>("frame_id", frame_id, "fcu");
		imu_nh.param("linear_acceleration_stdev", linear_stdev, 0.0003);// check default by MPU6000 spec
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
	geometry_msgs::Vector3 linear_accel_vec;
	boost::array<double, 9> linear_acceleration_cov;
	boost::array<double, 9> angular_velocity_cov;
	boost::array<double, 9> orientation_cov;
	boost::array<double, 9> unk_orientation_cov;
	boost::array<double, 9> magnetic_cov;

	static constexpr double GAUSS_TO_TESLA = 1.0e-4;
	static constexpr double MILLIT_TO_TESLA = 1000.0;
	static constexpr double MILLIRS_TO_RADSEC = 1.0e-3;
	static constexpr double MILLIG_TO_MS2 = 9.80665 / 1000.0;
	static constexpr double MILLIBAR_TO_PASCAL = 1.0e2;

	/* -*- helpers -*- */

	void setup_covariance(boost::array<double, 9> &cov, double stdev) {
		std::fill(cov.begin(), cov.end(), 0.0);
		if (stdev == 0.0)
			cov[0] = -1.0;
		else {
			cov[0 + 0] = cov[3 + 1] = cov[6 + 2] = std::pow(stdev, 2);
		}
	}

	void uas_store_attitude(tf::Quaternion &orientation,
			geometry_msgs::Vector3 &gyro_vec,
			geometry_msgs::Vector3 &acc_vec)
	{
		tf::Vector3 angular_velocity;
		tf::Vector3 linear_acceleration;
		tf::vector3MsgToTF(gyro_vec, angular_velocity);
		tf::vector3MsgToTF(acc_vec, linear_acceleration);

		uas->update_attitude_imu(orientation, angular_velocity, linear_acceleration);
	}

	//! fill imu/data message
	void fill_imu_msg_attitude(sensor_msgs::Imu::Ptr &imu_msg,
			tf::Quaternion &orientation,
			double xg, double yg, double zg)
	{
		tf::quaternionTFToMsg(orientation, imu_msg->orientation);

		imu_msg->angular_velocity.x = xg;
		imu_msg->angular_velocity.y = yg;
		imu_msg->angular_velocity.z = zg;

		// vector from HIGHRES_IMU or RAW_IMU
		imu_msg->linear_acceleration = linear_accel_vec;

		imu_msg->orientation_covariance = orientation_cov;
		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;
	}

	//! fill imu/data_raw message, store linear acceleration for imu/data
	void fill_imu_msg_raw(sensor_msgs::Imu::Ptr &imu_msg,
			double xg, double yg, double zg,
			double xa, double ya, double za)
	{
		imu_msg->angular_velocity.x = xg;
		imu_msg->angular_velocity.y = yg;
		imu_msg->angular_velocity.z = zg;

		imu_msg->linear_acceleration.x = xa;
		imu_msg->linear_acceleration.y = ya;
		imu_msg->linear_acceleration.z = za;
		linear_accel_vec = imu_msg->linear_acceleration;

		imu_msg->orientation_covariance = unk_orientation_cov;
		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;
	}

	/* -*- message handlers -*- */

	void handle_attitude(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (has_att_quat)
			return;

		mavlink_attitude_t att;
		mavlink_msg_attitude_decode(msg, &att);

		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();

		// NED -> ENU (body-fixed)
		tf::Quaternion orientation = tf::createQuaternionFromRPY(
				att.roll, -att.pitch, -att.yaw);

		fill_imu_msg_attitude(imu_msg, orientation,
				att.rollspeed,
				-att.pitchspeed,
				-att.yawspeed);

		uas_store_attitude(orientation,
				imu_msg->angular_velocity,
				imu_msg->linear_acceleration);

		// publish data
		imu_msg->header.frame_id = frame_id;
		imu_msg->header.stamp = uas->synchronise_stamp(att.time_boot_ms);
		imu_pub.publish(imu_msg);
	}

	// almost the same as handle_attitude(), but for ATTITUDE_QUATERNION
	void handle_attitude_quaternion(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_attitude_quaternion_t att_q;
		mavlink_msg_attitude_quaternion_decode(msg, &att_q);

		ROS_INFO_COND_NAMED(!has_att_quat, "imu", "Attitude quaternion IMU detected!");
		has_att_quat = true;

		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();

		// PX4 NED (w x y z) -> ROS ENU (x -y -z w) (body-fixed)
		tf::Quaternion orientation(att_q.q2, -att_q.q3, -att_q.q4, att_q.q1);

		fill_imu_msg_attitude(imu_msg, orientation,
				att_q.rollspeed,
				-att_q.pitchspeed,
				-att_q.yawspeed);

		uas_store_attitude(orientation,
				imu_msg->angular_velocity,
				imu_msg->linear_acceleration);

		// publish data
		imu_msg->header.frame_id = frame_id;
		imu_msg->header.stamp = uas->synchronise_stamp(att_q.time_boot_ms);
		imu_pub.publish(imu_msg);
	}

	void handle_highres_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_highres_imu_t imu_hr;
		mavlink_msg_highres_imu_decode(msg, &imu_hr);

		ROS_INFO_COND_NAMED(!has_hr_imu, "imu", "High resolution IMU detected!");
		has_hr_imu = true;

		std_msgs::Header header;
		header.stamp = uas->synchronise_stamp(imu_hr.time_usec);
		header.frame_id = frame_id;

		/* imu/data_raw filled by HR IMU */
		if (imu_hr.fields_updated & ((7 << 3) | (7 << 0))) {
			auto imu_msg = boost::make_shared<sensor_msgs::Imu>();

			fill_imu_msg_raw(imu_msg,
					imu_hr.xgyro, -imu_hr.ygyro, -imu_hr.zgyro,
					imu_hr.xacc, -imu_hr.yacc, -imu_hr.zacc);

			imu_msg->header = header;
			imu_raw_pub.publish(imu_msg);
		}

		if (imu_hr.fields_updated & (7 << 6)) {
			auto magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

			// Convert from local NED plane to ENU
			magn_msg->magnetic_field.x = imu_hr.ymag * GAUSS_TO_TESLA;
			magn_msg->magnetic_field.y = imu_hr.xmag * GAUSS_TO_TESLA;
			magn_msg->magnetic_field.z = -imu_hr.zmag * GAUSS_TO_TESLA;

			magn_msg->magnetic_field_covariance = magnetic_cov;

			magn_msg->header = header;
			magn_pub.publish(magn_msg);
		}

		if (imu_hr.fields_updated & (1 << 9)) {
			auto atmp_msg = boost::make_shared<sensor_msgs::FluidPressure>();

			atmp_msg->fluid_pressure = imu_hr.abs_pressure * MILLIBAR_TO_PASCAL;
			atmp_msg->header = header;
			press_pub.publish(atmp_msg);
		}

		if (imu_hr.fields_updated & (1 << 12)) {
			auto temp_msg = boost::make_shared<sensor_msgs::Temperature>();

			temp_msg->temperature = imu_hr.temperature;
			temp_msg->header = header;
			temp_pub.publish(temp_msg);
		}
	}

	void handle_raw_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (has_hr_imu || has_scaled_imu)
			return;

		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
		mavlink_raw_imu_t imu_raw;
		mavlink_msg_raw_imu_decode(msg, &imu_raw);

		std_msgs::Header header;
		header.stamp = uas->synchronise_stamp(imu_raw.time_usec);
		header.frame_id = frame_id;

		/* NOTE: APM send SCALED_IMU data as RAW_IMU */
		fill_imu_msg_raw(imu_msg,
				imu_raw.xgyro * MILLIRS_TO_RADSEC,
				-imu_raw.ygyro * MILLIRS_TO_RADSEC,
				-imu_raw.zgyro * MILLIRS_TO_RADSEC,
				imu_raw.xacc * MILLIG_TO_MS2,
				-imu_raw.yacc * MILLIG_TO_MS2,
				-imu_raw.zacc * MILLIG_TO_MS2);

		if (!uas->is_ardupilotmega()) {
			ROS_WARN_THROTTLE_NAMED(60, "imu", "RAW_IMU: linear acceleration known on APM only");
			linear_accel_vec.x = 0.0;
			linear_accel_vec.y = 0.0;
			linear_accel_vec.z = 0.0;
		}

		imu_msg->header = header;
		imu_raw_pub.publish(imu_msg);

		/* -*- magnetic vector -*- */
		auto magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

		// Convert from local NED plane to ENU
		magn_msg->magnetic_field.x = imu_raw.ymag * MILLIT_TO_TESLA;
		magn_msg->magnetic_field.y = imu_raw.xmag * MILLIT_TO_TESLA;
		magn_msg->magnetic_field.z = -imu_raw.zmag * MILLIT_TO_TESLA;

		magn_msg->magnetic_field_covariance = magnetic_cov;

		magn_msg->header = header;
		magn_pub.publish(magn_msg);
	}

	void handle_scaled_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (has_hr_imu)
			return;

		ROS_INFO_COND_NAMED(!has_scaled_imu, "imu", "Scaled IMU message used.");
		has_scaled_imu = true;

		auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
		mavlink_scaled_imu_t imu_raw;
		mavlink_msg_scaled_imu_decode(msg, &imu_raw);

		std_msgs::Header header;
		header.stamp = uas->synchronise_stamp(imu_raw.time_boot_ms);

		fill_imu_msg_raw(imu_msg,
				imu_raw.xgyro * MILLIRS_TO_RADSEC,
				-imu_raw.ygyro * MILLIRS_TO_RADSEC,
				-imu_raw.zgyro * MILLIRS_TO_RADSEC,
				imu_raw.xacc * MILLIG_TO_MS2,
				-imu_raw.yacc * MILLIG_TO_MS2,
				-imu_raw.zacc * MILLIG_TO_MS2);

		imu_msg->header = header;
		imu_raw_pub.publish(imu_msg);

		/* -*- magnetic vector -*- */
		auto magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

		// Convert from local NED plane to ENU
		magn_msg->magnetic_field.x = imu_raw.ymag * MILLIT_TO_TESLA;
		magn_msg->magnetic_field.y = imu_raw.xmag * MILLIT_TO_TESLA;
		magn_msg->magnetic_field.z = -imu_raw.zmag * MILLIT_TO_TESLA;

		magn_msg->magnetic_field_covariance = magnetic_cov;

		magn_msg->header = header;
		magn_pub.publish(magn_msg);
	}

	void handle_scaled_pressure(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (has_hr_imu)
			return;

		mavlink_scaled_pressure_t press;
		mavlink_msg_scaled_pressure_decode(msg, &press);

		std_msgs::Header header;
		header.stamp = uas->synchronise_stamp(press.time_boot_ms);
		header.frame_id = frame_id;

		auto temp_msg = boost::make_shared<sensor_msgs::Temperature>();
		temp_msg->temperature = press.temperature / 100.0;
		temp_msg->header = header;
		temp_pub.publish(temp_msg);

		auto atmp_msg = boost::make_shared<sensor_msgs::FluidPressure>();
		atmp_msg->fluid_pressure = press.press_abs * 100.0;
		atmp_msg->header = header;
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


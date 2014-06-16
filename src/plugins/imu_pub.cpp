/**
 * @brief IMU publish plugin
 * @file imu_pub.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
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

class IMUPubPlugin : public MavRosPlugin {
public:
	IMUPubPlugin() :
		linear_accel_vec(),
		has_hr_imu(false),
		has_scaled_imu(false),
		gauss_to_tesla(1.0e-4),
		millit_to_tesla(1000.0),
		millirs_to_radsec(1000.0),
		millig_to_ms2(9.80665 / 1000.00),
		millibar_to_pascal(1.0e5)
	{
	};

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		double linear_stdev, angular_stdev, orientation_stdev;

		uas = &uas_;

		nh.param<std::string>("imu/frame_id", frame_id, "fcu");
		nh.param("imu/linear_acceleration_stdev", linear_stdev, 0.0003); // check default by MPU6000 spec
		nh.param("imu/angular_velocity_stdev", angular_stdev, 0.02 * (M_PI / 180.0)); // check default by MPU6000 spec
		nh.param("imu/orientation_stdev", orientation_stdev, 1.0);

		std::fill(linear_acceleration_cov.begin(),
				linear_acceleration_cov.end(),
				0.0);
		linear_acceleration_cov[0+0] =
			linear_acceleration_cov[3+1] =
			linear_acceleration_cov[6+2] = std::pow(linear_stdev, 2);

		std::fill(angular_velocity_cov.begin(),
				angular_velocity_cov.end(),
				0.0);
		angular_velocity_cov[0+0] =
			angular_velocity_cov[3+1] =
			angular_velocity_cov[6+2] = std::pow(angular_stdev, 2);

		std::fill(orientation_cov.begin(),
				orientation_cov.end(),
				0.0);
		orientation_cov[0+0] =
			orientation_cov[3+1] =
			orientation_cov[6+2] = std::pow(orientation_stdev, 2);

		imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
		magn_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
		temp_pub = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 10);
		press_pub = nh.advertise<sensor_msgs::FluidPressure>("imu/atm_pressure", 10);
		imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
	}

	std::string get_name() {
		return "IMUPub";
	}

	std::vector<uint8_t> get_supported_messages() {
		return {
			MAVLINK_MSG_ID_ATTITUDE,
			MAVLINK_MSG_ID_RAW_IMU,
			MAVLINK_MSG_ID_SCALED_IMU,
			MAVLINK_MSG_ID_HIGHRES_IMU,
			MAVLINK_MSG_ID_SCALED_PRESSURE
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_ATTITUDE:
			handle_attitude(msg);
			break;
		case MAVLINK_MSG_ID_HIGHRES_IMU:
			handle_highres_imu(msg);
			break;
		case MAVLINK_MSG_ID_RAW_IMU:
			handle_raw_imu(msg);
			break;
		case MAVLINK_MSG_ID_SCALED_IMU:
			handle_scaled_imu(msg);
			break;
		case MAVLINK_MSG_ID_SCALED_PRESSURE:
			handle_scaled_pressure(msg);
			break;
		};
	}

private:
	std::string frame_id;
	UAS *uas;

	ros::Publisher imu_pub;
	ros::Publisher imu_raw_pub;
	ros::Publisher magn_pub;
	ros::Publisher temp_pub;
	ros::Publisher press_pub;

	bool has_hr_imu;
	bool has_scaled_imu;
	geometry_msgs::Vector3 linear_accel_vec;
	boost::array<double, 9> linear_acceleration_cov;
	boost::array<double, 9> angular_velocity_cov;
	boost::array<double, 9> orientation_cov;

	const double gauss_to_tesla;		// = 1.0e-4;
	const double millit_to_tesla;		// = 1000.0;
	const double millirs_to_radsec;		// = 1000.0;
	const double millig_to_ms2;		// = 9.80665 / 1000.0;
	const double millibar_to_pascal;	// = 1.0e5;


	void handle_attitude(const mavlink_message_t *msg) {
		if (imu_pub.getNumSubscribers() == 0)
			return;

		mavlink_attitude_t att;
		mavlink_msg_attitude_decode(msg, &att);

		sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);

		// TODO: check/verify that these are body-fixed
		imu_msg->orientation = tf::createQuaternionMsgFromRollPitchYaw(
				att.roll, -att.pitch, -att.yaw);

		imu_msg->angular_velocity.x = att.rollspeed;
		imu_msg->angular_velocity.y = -att.pitchspeed;
		imu_msg->angular_velocity.z = -att.yawspeed;

		// vector from HIGHRES_IMU or RAW_IMU
		imu_msg->linear_acceleration = linear_accel_vec;

		imu_msg->orientation_covariance = orientation_cov;
		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

		imu_msg->header.frame_id = frame_id;
		imu_msg->header.stamp = ros::Time::now();
		imu_pub.publish(imu_msg);
	}

	void fill_imu_msg_vec(sensor_msgs::ImuPtr &imu_msg,
			double xg, double yg, double zg,
			double xa, double ya, double za)
	{
		imu_msg->angular_velocity.x = xg;
		imu_msg->angular_velocity.y = yg;
		imu_msg->angular_velocity.z = xg;

		imu_msg->linear_acceleration.x = xa;
		imu_msg->linear_acceleration.y = ya;
		imu_msg->linear_acceleration.z = za;
	}

	void handle_highres_imu(const mavlink_message_t *msg) {
		mavlink_highres_imu_t imu_hr;
		mavlink_msg_highres_imu_decode(msg, &imu_hr);

		ROS_INFO_COND_NAMED(!has_hr_imu, "imu", "High resolution IMU detected!");
		has_hr_imu = true;

		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = frame_id;

		/* imu/data_raw filled by HR IMU */
		if (imu_raw_pub.getNumSubscribers() > 0 &&
				imu_hr.fields_updated & 0x003f) {
			sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);

			fill_imu_msg_vec(imu_msg,
					imu_hr.xgyro, -imu_hr.ygyro, -imu_hr.zgyro,
					imu_hr.xacc, -imu_hr.yacc, -imu_hr.zacc);
			linear_accel_vec = imu_msg->linear_acceleration;

			std::fill(imu_msg->orientation_covariance.begin(),
					imu_msg->orientation_covariance.end(), 0);
			imu_msg->orientation_covariance[0] = -1;

			imu_msg->angular_velocity_covariance = angular_velocity_cov;
			imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

			imu_msg->header = header;
			imu_raw_pub.publish(imu_msg);
		}

		if (magn_pub.getNumSubscribers() > 0 &&
				imu_hr.fields_updated & 0x01c0) {
			sensor_msgs::MagneticFieldPtr magn_msg(new sensor_msgs::MagneticField);

			magn_msg->magnetic_field.x = imu_hr.xmag * gauss_to_tesla;
			magn_msg->magnetic_field.y = imu_hr.ymag * gauss_to_tesla;
			magn_msg->magnetic_field.z = imu_hr.zmag * gauss_to_tesla;

			// TODO: again covariance
			std::fill(magn_msg->magnetic_field_covariance.begin(),
					magn_msg->magnetic_field_covariance.end(), 0);
			magn_msg->magnetic_field_covariance[0] = -1;

			magn_msg->header = header;
			magn_pub.publish(magn_msg);
		}

		if (press_pub.getNumSubscribers() > 0 &&
				imu_hr.fields_updated & 0x0e00) {
			sensor_msgs::FluidPressurePtr atmp_msg(new sensor_msgs::FluidPressure);

			atmp_msg->fluid_pressure = imu_hr.abs_pressure * millibar_to_pascal;
			atmp_msg->header = header;
			press_pub.publish(atmp_msg);
		}

		if (temp_pub.getNumSubscribers() > 0 &&
				imu_hr.fields_updated & 0x1000) {
			sensor_msgs::TemperaturePtr temp_msg(new sensor_msgs::Temperature);

			temp_msg->temperature = imu_hr.temperature;
			temp_msg->header = header;
			temp_pub.publish(temp_msg);
		}
	}

	void handle_raw_imu(const mavlink_message_t *msg) {
		if (has_hr_imu || has_scaled_imu)
			return;

		sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
		mavlink_raw_imu_t imu_raw;
		mavlink_msg_raw_imu_decode(msg, &imu_raw);

		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = frame_id;

		/* NOTE: APM send SCALED_IMU data as RAW_IMU */
		fill_imu_msg_vec(imu_msg,
				imu_raw.xgyro * millirs_to_radsec,
				-imu_raw.ygyro * millirs_to_radsec,
				-imu_raw.zgyro * millirs_to_radsec,
				imu_raw.xacc * millig_to_ms2,
				-imu_raw.yacc * millig_to_ms2,
				-imu_raw.zacc * millig_to_ms2);

		if (!uas->is_ardupilotmega()) {
			ROS_WARN_THROTTLE_NAMED(60, "imu", "RAW_IMU: linear acceleration known on APM only");
			linear_accel_vec.x = 0;
			linear_accel_vec.y = 0;
			linear_accel_vec.z = 0;
		} else
			linear_accel_vec = imu_msg->linear_acceleration;

		std::fill(imu_msg->orientation_covariance.begin(),
				imu_msg->orientation_covariance.end(), 0);
		imu_msg->orientation_covariance[0] = -1;

		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

		imu_msg->header = header;
		imu_raw_pub.publish(imu_msg);

		/* -*- magnetic vector -*- */
		sensor_msgs::MagneticFieldPtr magn_msg(new sensor_msgs::MagneticField);

		magn_msg->magnetic_field.x = imu_raw.xmag * millit_to_tesla;
		magn_msg->magnetic_field.y = -imu_raw.ymag * millit_to_tesla;
		magn_msg->magnetic_field.z = -imu_raw.zmag * millit_to_tesla;

		// TODO: again covariance
		std::fill(magn_msg->magnetic_field_covariance.begin(),
				magn_msg->magnetic_field_covariance.end(), 0);
		magn_msg->magnetic_field_covariance[0] = -1;

		magn_msg->header = header;
		magn_pub.publish(magn_msg);
	}

	void handle_scaled_imu(const mavlink_message_t *msg) {
		if (has_hr_imu)
			return;

		ROS_INFO_COND_NAMED(!has_scaled_imu, "imu", "Scaled IMU message used.");
		has_scaled_imu = true;

		sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
		mavlink_scaled_imu_t imu_raw;
		mavlink_msg_scaled_imu_decode(msg, &imu_raw);

		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = frame_id;

		fill_imu_msg_vec(imu_msg,
				imu_raw.xgyro * millirs_to_radsec,
				-imu_raw.ygyro * millirs_to_radsec,
				-imu_raw.zgyro * millirs_to_radsec,
				imu_raw.xacc * millig_to_ms2,
				-imu_raw.yacc * millig_to_ms2,
				-imu_raw.zacc * millig_to_ms2);
		linear_accel_vec = imu_msg->linear_acceleration;

		std::fill(imu_msg->orientation_covariance.begin(),
				imu_msg->orientation_covariance.end(), 0);
		imu_msg->orientation_covariance[0] = -1;

		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

		imu_msg->header = header;
		imu_raw_pub.publish(imu_msg);

		/* -*- magnetic vector -*- */
		sensor_msgs::MagneticFieldPtr magn_msg(new sensor_msgs::MagneticField);

		magn_msg->magnetic_field.x = imu_raw.xmag * millit_to_tesla;
		magn_msg->magnetic_field.y = -imu_raw.ymag * millit_to_tesla;
		magn_msg->magnetic_field.z = -imu_raw.zmag * millit_to_tesla;

		// TODO: again covariance
		std::fill(magn_msg->magnetic_field_covariance.begin(),
				magn_msg->magnetic_field_covariance.end(), 0);
		magn_msg->magnetic_field_covariance[0] = -1;

		magn_msg->header = header;
		magn_pub.publish(magn_msg);
	}

	void handle_scaled_pressure(const mavlink_message_t *msg) {
		if (has_hr_imu)
			return;

		mavlink_scaled_pressure_t press;
		mavlink_msg_scaled_pressure_decode(msg, &press);

		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = frame_id;

		sensor_msgs::TemperaturePtr temp_msg(new sensor_msgs::Temperature);
		temp_msg->temperature = press.temperature / 100.0;
		temp_msg->header = header;
		temp_pub.publish(temp_msg);

		sensor_msgs::FluidPressurePtr atmp_msg(new sensor_msgs::FluidPressure);
		atmp_msg->fluid_pressure = press.press_abs * 100.0;
		atmp_msg->header = header;
		press_pub.publish(atmp_msg);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::IMUPubPlugin, mavplugin::MavRosPlugin)


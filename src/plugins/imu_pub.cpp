/**
 * @brief IMU publish plugin
 * @file imu_pub.cpp
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
		linear_accel_vec(),
		has_hr_imu(false),
		has_scaled_imu(false)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		double linear_stdev, angular_stdev, orientation_stdev, mag_stdev;

		uas = &uas_;

		nh.param<std::string>("imu/frame_id", frame_id, "fcu");
		nh.param("imu/linear_acceleration_stdev", linear_stdev, 0.0003); // check default by MPU6000 spec
		nh.param("imu/angular_velocity_stdev", angular_stdev, 0.02 * (M_PI / 180.0)); // check default by MPU6000 spec
		nh.param("imu/orientation_stdev", orientation_stdev, 1.0);
		nh.param("imu/magnetic_stdev", mag_stdev, 0.0);

		setup_covariance(linear_acceleration_cov, linear_stdev);
		setup_covariance(angular_velocity_cov, angular_stdev);
		setup_covariance(orientation_cov, orientation_stdev);
		setup_covariance(magnetic_cov, mag_stdev);
		setup_covariance(unk_orientation_cov, 0.0);

		imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
		magn_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
		temp_pub = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 10);
		press_pub = nh.advertise<sensor_msgs::FluidPressure>("imu/atm_pressure", 10);
		imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
	}

	std::string const get_name() const {
		return "IMUPub";
	}

	std::vector<uint8_t> const get_supported_messages() const {
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
	boost::array<double, 9> unk_orientation_cov;
	boost::array<double, 9> magnetic_cov;

	static constexpr double GAUSS_TO_TESLA = 1.0e-4;
	static constexpr double MILLIT_TO_TESLA = 1000.0;
	static constexpr double MILLIRS_TO_RADSEC = 1000.0;
	static constexpr double MILLIG_TO_MS2 = 9.80665 / 1000.0;
	static constexpr double MILLIBAR_TO_PASCAL = 1.0e5;


	void setup_covariance(boost::array<double, 9> &cov, double stdev) {
		std::fill(cov.begin(), cov.end(), 0.0);
		if (stdev == 0.0)
			cov[0] = -1.0;
		else {
			cov[0+0] = cov[3+1] = cov[6+2] = std::pow(stdev, 2);
		}
	}

	void handle_attitude(const mavlink_message_t *msg) {
		mavlink_attitude_t att;
		mavlink_msg_attitude_decode(msg, &att);

		sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();

		// TODO: check/verify that these are body-fixed
		tf::Quaternion orientation = tf::createQuaternionFromRPY(
				att.roll, -att.pitch, -att.yaw);
		tf::quaternionTFToMsg(orientation, imu_msg->orientation);

		imu_msg->angular_velocity.x = att.rollspeed;
		imu_msg->angular_velocity.y = -att.pitchspeed;
		imu_msg->angular_velocity.z = -att.yawspeed;

		// store gyro data to UAS
		// vector from HIGHRES_IMU or RAW_IMU
		imu_msg->linear_acceleration = linear_accel_vec;

		imu_msg->orientation_covariance = orientation_cov;
		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

		// store data in UAS
		tf::Vector3 angular_velocity;
		tf::Vector3 linear_acceleration;
		tf::vector3MsgToTF(imu_msg->angular_velocity, angular_velocity);
		tf::vector3MsgToTF(imu_msg->linear_acceleration, linear_acceleration);

		uas->set_attitude_orientation(orientation);
		uas->set_attitude_angular_velocity(angular_velocity);
		uas->set_attitude_linear_acceleration(linear_acceleration);

		// publish data
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
		imu_msg->angular_velocity.z = zg;

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
		if (imu_hr.fields_updated & 0x003f) {
			sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();

			fill_imu_msg_vec(imu_msg,
					imu_hr.xgyro, -imu_hr.ygyro, -imu_hr.zgyro,
					imu_hr.xacc, -imu_hr.yacc, -imu_hr.zacc);
			linear_accel_vec = imu_msg->linear_acceleration;

			imu_msg->orientation_covariance = unk_orientation_cov;
			imu_msg->angular_velocity_covariance = angular_velocity_cov;
			imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

			imu_msg->header = header;
			imu_raw_pub.publish(imu_msg);
		}

		if (imu_hr.fields_updated & 0x01c0) {
			sensor_msgs::MagneticFieldPtr magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

			// Convert from local NED plane to ENU
			magn_msg->magnetic_field.x = imu_hr.ymag * GAUSS_TO_TESLA;
			magn_msg->magnetic_field.y = imu_hr.xmag * GAUSS_TO_TESLA;
			magn_msg->magnetic_field.z = -imu_hr.zmag * GAUSS_TO_TESLA;

			magn_msg->magnetic_field_covariance = magnetic_cov;

			magn_msg->header = header;
			magn_pub.publish(magn_msg);
		}

		if (imu_hr.fields_updated & 0x0e00) {
			sensor_msgs::FluidPressurePtr atmp_msg = boost::make_shared<sensor_msgs::FluidPressure>();

			atmp_msg->fluid_pressure = imu_hr.abs_pressure * MILLIBAR_TO_PASCAL;
			atmp_msg->header = header;
			press_pub.publish(atmp_msg);
		}

		if (imu_hr.fields_updated & 0x1000) {
			sensor_msgs::TemperaturePtr temp_msg = boost::make_shared<sensor_msgs::Temperature>();

			temp_msg->temperature = imu_hr.temperature;
			temp_msg->header = header;
			temp_pub.publish(temp_msg);
		}
	}

	void handle_raw_imu(const mavlink_message_t *msg) {
		if (has_hr_imu || has_scaled_imu)
			return;

		sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();
		mavlink_raw_imu_t imu_raw;
		mavlink_msg_raw_imu_decode(msg, &imu_raw);

		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = frame_id;

		/* NOTE: APM send SCALED_IMU data as RAW_IMU */
		fill_imu_msg_vec(imu_msg,
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
		} else
			linear_accel_vec = imu_msg->linear_acceleration;

		imu_msg->orientation_covariance = unk_orientation_cov;
		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

		imu_msg->header = header;
		imu_raw_pub.publish(imu_msg);

		/* -*- magnetic vector -*- */
		sensor_msgs::MagneticFieldPtr magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

		// Convert from local NED plane to ENU
		magn_msg->magnetic_field.x = imu_raw.ymag * MILLIT_TO_TESLA;
		magn_msg->magnetic_field.y = imu_raw.xmag * MILLIT_TO_TESLA;
		magn_msg->magnetic_field.z = -imu_raw.zmag * MILLIT_TO_TESLA;

		magn_msg->magnetic_field_covariance = magnetic_cov;

		magn_msg->header = header;
		magn_pub.publish(magn_msg);
	}

	void handle_scaled_imu(const mavlink_message_t *msg) {
		if (has_hr_imu)
			return;

		ROS_INFO_COND_NAMED(!has_scaled_imu, "imu", "Scaled IMU message used.");
		has_scaled_imu = true;

		sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();
		mavlink_scaled_imu_t imu_raw;
		mavlink_msg_scaled_imu_decode(msg, &imu_raw);

		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = frame_id;

		fill_imu_msg_vec(imu_msg,
				imu_raw.xgyro * MILLIRS_TO_RADSEC,
				-imu_raw.ygyro * MILLIRS_TO_RADSEC,
				-imu_raw.zgyro * MILLIRS_TO_RADSEC,
				imu_raw.xacc * MILLIG_TO_MS2,
				-imu_raw.yacc * MILLIG_TO_MS2,
				-imu_raw.zacc * MILLIG_TO_MS2);
		linear_accel_vec = imu_msg->linear_acceleration;

		imu_msg->orientation_covariance = unk_orientation_cov;
		imu_msg->angular_velocity_covariance = angular_velocity_cov;
		imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

		imu_msg->header = header;
		imu_raw_pub.publish(imu_msg);

		/* -*- magnetic vector -*- */
		sensor_msgs::MagneticFieldPtr magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

		// Convert from local NED plane to ENU
		magn_msg->magnetic_field.x = imu_raw.ymag * MILLIT_TO_TESLA;
		magn_msg->magnetic_field.y = imu_raw.xmag * MILLIT_TO_TESLA;
		magn_msg->magnetic_field.z = -imu_raw.zmag * MILLIT_TO_TESLA;

		magn_msg->magnetic_field_covariance = magnetic_cov;

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

		sensor_msgs::TemperaturePtr temp_msg = boost::make_shared<sensor_msgs::Temperature>();
		temp_msg->temperature = press.temperature / 100.0;
		temp_msg->header = header;
		temp_pub.publish(temp_msg);

		sensor_msgs::FluidPressurePtr atmp_msg = boost::make_shared<sensor_msgs::FluidPressure>();
		atmp_msg->fluid_pressure = press.press_abs * 100.0;
		atmp_msg->header = header;
		press_pub.publish(atmp_msg);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::IMUPubPlugin, mavplugin::MavRosPlugin)


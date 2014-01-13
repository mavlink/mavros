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

namespace mavplugin {

class IMUPubPlugin : public MavRosPlugin {
public:
	IMUPubPlugin()
	{
		imu_raw = {
			0,
			0.0, 0.0, 0.0, // accel
			0.0, 0.0, 0.0, // gyro
			0.0, 0.0, 0.0, // magn
			0.0, 0.0, 0.0, // pressure
			0.0,           // temp
			0
		};
	};

	void initialize(ros::NodeHandle &nh,
			const boost::shared_ptr<mavconn::MAVConnInterface> &mav_link,
			diagnostic_updater::Updater &diag_updater,
			MavContext &context)
	{
		double linear_stdev, angular_stdev, orientation_stdev;

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
		temp_pub = nh.advertise<sensor_msgs::Imu>("imu/temperature", 10);
		imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
	}

	std::string get_name() {
		return "IMUPub";
	}

	std::vector<uint8_t> get_supported_messages() {
		return {
			MAVLINK_MSG_ID_ATTITUDE,
			MAVLINK_MSG_ID_HIGHRES_IMU
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_ATTITUDE:
			if (imu_pub.getNumSubscribers() > 0) {
				mavlink_attitude_t att;
				mavlink_msg_attitude_decode(msg, &att);

				sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);

				// TODO: check/verify that these are body-fixed
				imu_msg->orientation = tf::createQuaternionMsgFromRollPitchYaw(
						att.roll, -att.pitch, -att.yaw);

				imu_msg->angular_velocity.x = att.rollspeed;
				imu_msg->angular_velocity.y = -att.pitchspeed;
				imu_msg->angular_velocity.z = -att.yawspeed;

				imu_msg->linear_acceleration.x = imu_raw.xacc;
				imu_msg->linear_acceleration.y = -imu_raw.yacc;
				imu_msg->linear_acceleration.z = -imu_raw.zacc;

				imu_msg->orientation_covariance = orientation_cov;
				imu_msg->angular_velocity_covariance = angular_velocity_cov;
				imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

				imu_msg->header.frame_id = frame_id;
				imu_msg->header.seq = imu_raw.time_usec / 1000;
				imu_msg->header.stamp = ros::Time::now();

				imu_pub.publish(imu_msg);
			}
			break;

		case MAVLINK_MSG_ID_HIGHRES_IMU:
			{
				mavlink_msg_highres_imu_decode(msg, &imu_raw);

				std_msgs::Header header;
				header.stamp = ros::Time::now();
				header.seq = imu_raw.time_usec / 1000;
				header.frame_id = frame_id;

				if (imu_raw_pub.getNumSubscribers() > 0 &&
						imu_raw.fields_updated & 0x003f) {
					sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);

					imu_msg->angular_velocity.x = imu_raw.xgyro;
					imu_msg->angular_velocity.y = -imu_raw.ygyro;
					imu_msg->angular_velocity.z = -imu_raw.xgyro;

					imu_msg->linear_acceleration.x = imu_raw.xacc;
					imu_msg->linear_acceleration.y = -imu_raw.yacc;
					imu_msg->linear_acceleration.z = -imu_raw.zacc;

					std::fill(imu_msg->orientation_covariance.begin(),
							imu_msg->orientation_covariance.end(), 0);
					imu_msg->orientation_covariance[0] = -1;

					imu_msg->angular_velocity_covariance = angular_velocity_cov;
					imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

					imu_msg->header = header;
					imu_raw_pub.publish(imu_msg);
				}
				if (magn_pub.getNumSubscribers() > 0 &&
						imu_raw.fields_updated & 0x01c0) {
					const double gauss_to_tesla = 1.0e-4;
					sensor_msgs::MagneticFieldPtr magn_msg(new sensor_msgs::MagneticField);

					magn_msg->magnetic_field.x = imu_raw.xmag * gauss_to_tesla;
					magn_msg->magnetic_field.y = imu_raw.ymag * gauss_to_tesla;
					magn_msg->magnetic_field.z = imu_raw.zmag * gauss_to_tesla;

					// TODO: again covariance
					std::fill(magn_msg->magnetic_field_covariance.begin(),
							magn_msg->magnetic_field_covariance.end(), 0);

					magn_msg->header = header;
					magn_pub.publish(magn_msg);
				}
				if (imu_raw.fields_updated & 0x0e00) {
					/* TODO: pressure & alt */
				}
				if (temp_pub.getNumSubscribers() > 0 &&
						imu_raw.fields_updated & 0x1000) {
					sensor_msgs::TemperaturePtr temp_msg(new sensor_msgs::Temperature);

					temp_msg->temperature = imu_raw.temperature;
					temp_msg->header = header;
					temp_pub.publish(temp_msg);
				}
			}
			break;
		};
	}

private:
	std::string frame_id;

	ros::Publisher imu_pub;
	ros::Publisher imu_raw_pub;
	ros::Publisher magn_pub;
	ros::Publisher temp_pub;

	mavlink_highres_imu_t imu_raw;
	boost::array<double, 9> linear_acceleration_cov;
	boost::array<double, 9> angular_velocity_cov;
	boost::array<double, 9> orientation_cov;
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::IMUPubPlugin, mavplugin::MavRosPlugin)


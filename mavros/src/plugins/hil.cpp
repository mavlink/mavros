/**
 * @brief HilControls plugin
 * @file hil_controls.cpp
 * @author Pavel Vechersky <pvechersky@student.ethz.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016,2017 Pavel Vechersky, Beat Küng, Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/HilControls.h>
#include <mavros_msgs/HilActuatorControls.h>
#include <mavros_msgs/HilStateQuaternion.h>
#include <mavros_msgs/HilGPS.h>
#include <mavros_msgs/HilSensor.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Hil plugin
 */
class HilPlugin : public plugin::PluginBase {
public:
	HilPlugin() : PluginBase(),
		hil_nh("~hil")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		last_time = ros::Time(0.0);
		period = ros::Duration(0.025);	// 40hz

		hilStateQuaternion_sub = hil_nh.subscribe("hil_state", 10, &HilPlugin::state_quat_cb, this);
		hilGPS_sub = hil_nh.subscribe("hilgps", 10, &HilPlugin::gps_cb, this);
		hilSensor_sub = hil_nh.subscribe("imu_ned", 10, &HilPlugin::sensor_cb, this);

		hil_controls_pub = hil_nh.advertise<mavros_msgs::HilControls>("hil_controls", 10);
		hil_actuator_controls_pub = hil_nh.advertise<mavros_msgs::HilActuatorControls>("hil_actuator_controls", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&HilPlugin::handle_hil_controls),
			       make_handler(&HilPlugin::handle_hil_actuator_controls),
		};
	}

private:
	ros::NodeHandle hil_nh;

	ros::Publisher hil_controls_pub;
	ros::Publisher hil_actuator_controls_pub;

	ros::Subscriber hilStateQuaternion_sub;
	ros::Subscriber hilGPS_sub;
	ros::Subscriber hilSensor_sub;

	ros::Time last_time;
	ros::Duration period;

	/* -*- rx handlers -*- */

	void handle_hil_controls(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HIL_CONTROLS &hil_controls) {
		// Throttle incoming messages to 40hz
		if ((ros::Time::now() - last_time) < period) {
			return;
		}
		last_time = ros::Time::now();

		auto hil_controls_msg = boost::make_shared<mavros_msgs::HilControls>();

		hil_controls_msg->header.stamp = m_uas->synchronise_stamp(hil_controls.time_usec);
		// [[[cog:
		// for f in (
		//     'roll_ailerons', 'pitch_elevator', 'yaw_rudder', 'throttle',
		//     'aux1', 'aux2', 'aux3', 'aux4', 'mode', 'nav_mode'):
		//     cog.outl("hil_controls_msg->%s = hil_controls.%s;" % (f, f))
		// ]]]
		hil_controls_msg->roll_ailerons = hil_controls.roll_ailerons;
		hil_controls_msg->pitch_elevator = hil_controls.pitch_elevator;
		hil_controls_msg->yaw_rudder = hil_controls.yaw_rudder;
		hil_controls_msg->throttle = hil_controls.throttle;
		hil_controls_msg->aux[0] = hil_controls.aux1;
		hil_controls_msg->aux[1] = hil_controls.aux2;
		hil_controls_msg->aux[2] = hil_controls.aux3;
		hil_controls_msg->aux[3] = hil_controls.aux4;
		hil_controls_msg->mode = hil_controls.mode;
		hil_controls_msg->nav_mode = hil_controls.nav_mode;
		// [[[end]]] (checksum: a2c87ee8f36e7a32b08be5e0fe665b5a)

		hil_controls_pub.publish(hil_controls_msg);
	}

	void handle_hil_actuator_controls(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HIL_ACTUATOR_CONTROLS &hil_actuator_controls) {
		auto hil_actuator_controls_msg = boost::make_shared<mavros_msgs::HilActuatorControls>();

		hil_actuator_controls_msg->header.stamp = m_uas->synchronise_stamp(hil_actuator_controls.time_usec);
		for (int i = 0; i < 16; ++i) {
			hil_actuator_controls_msg->controls[i] = hil_actuator_controls.controls[i];
		}
		hil_actuator_controls_msg->mode = hil_actuator_controls.mode;
		hil_actuator_controls_msg->flags = hil_actuator_controls.flags;

		hil_actuator_controls_pub.publish(hil_actuator_controls_msg);
	}

	//! Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_GPS
	void set_hil_gps(uint64_t time_boot_us, uint8_t fix_type,
				int32_t lat, int32_t lon, int32_t alt,
				uint16_t eph, uint16_t epv, uint16_t vel,
				int16_t vn, int16_t ve, int16_t vd,
				uint16_t cog,
				uint8_t satellites_visible) {
		mavlink::common::msg::HIL_GPS gps;

		// there is no target sys in this mavlink message!

		gps.time_usec = time_boot_us;
		gps.fix_type = fix_type;
		gps.lat = lat;
		gps.lon = lon;
		gps.alt = alt;
		gps.eph = eph;
		gps.epv = epv;
		gps.vel = vel;
		gps.vn = vn;
		gps.ve = ve;
		gps.vd = vd;
		gps.cog = cog;
		gps.satellites_visible = satellites_visible;

		UAS_FCU(m_uas)->send_message_ignore_drop(gps);
	}

	/* -*- low-level send -*- */

	//! Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_STATE_QUATERNION
	void set_hil_state_quaternion(uint64_t time_boot_us,
				float qw, float qx, float qy, float qz,
				float rollspeed, float pitchspeed, float yawspeed,
				int32_t lat, int32_t lon, int32_t alt,
				int16_t vx, int16_t vy, int16_t vz,
				uint16_t ind_airspeed, uint16_t true_airspeed,
				int16_t xacc, int16_t yacc, int16_t zacc) {
		mavlink::common::msg::HIL_STATE_QUATERNION state_quat;

		state_quat.time_usec = time_boot_us;

		state_quat.attitude_quaternion[0] = qw;
		state_quat.attitude_quaternion[1] = qx;
		state_quat.attitude_quaternion[2] = qy;
		state_quat.attitude_quaternion[3] = qz;

		state_quat.rollspeed = rollspeed;
		state_quat.pitchspeed = pitchspeed;
		state_quat.yawspeed = yawspeed;

		state_quat.lat = lat;
		state_quat.lon = lon;
		state_quat.alt = alt;

		state_quat.vx = vx;
		state_quat.vy = vy;
		state_quat.vz = vz;

		state_quat.ind_airspeed = ind_airspeed;
		state_quat.true_airspeed = true_airspeed;

		state_quat.xacc = xacc;
		state_quat.yacc = yacc;
		state_quat.zacc = zacc;

		UAS_FCU(m_uas)->send_message_ignore_drop(state_quat);
	}

	//! Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_SENSOR
	void set_hil_sensor(uint64_t time_boot_us,
				float xacc, float yacc, float zacc,
				float xgyro, float ygyro, float zgyro,
				float xmag, float ymag, float zmag,
				float abs_pressure, float diff_pressure, float pressure_alt,
				float temperature,
				uint32_t fields_updated) {
		mavlink::common::msg::HIL_SENSOR sensor;

		sensor.time_usec = time_boot_us;
		sensor.xacc = xacc;
		sensor.yacc = yacc;
		sensor.zacc = zacc;
		sensor.xgyro = xgyro;
		sensor.ygyro = ygyro;
		sensor.zgyro = zgyro;
		sensor.xmag = xmag;
		sensor.ymag = ymag;
		sensor.zmag = zmag;
		sensor.abs_pressure = abs_pressure;
		sensor.diff_pressure = diff_pressure;
		sensor.pressure_alt = pressure_alt;
		sensor.temperature = temperature;
		sensor.fields_updated = fields_updated;

		UAS_FCU(m_uas)->send_message_ignore_drop(sensor);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send hil_state_quaternion to FCU.
	 */
	void send_hil_state_quaternion(const ros::Time &stamp,
				float qw, float qx, float qy, float qz,
				float rollspeed, float pitchspeed, float yawspeed,
				int32_t lat, int32_t lon, int32_t alt,
				int16_t vx, int16_t vy, int16_t vz,
				uint16_t ind_airspeed, uint16_t true_airspeed,
				int16_t xacc, int16_t yacc, int16_t zacc) {
		set_hil_state_quaternion(stamp.toNSec() / 1000,
					qw, qx, qy, qz,
					rollspeed, pitchspeed, yawspeed,
					lat, lon, alt,
					vx, vy, vz,
					ind_airspeed,
					true_airspeed,
					xacc, yacc, zacc);
	}

	/**
	 * @brief Send hil_gps to FCU.
	 */
	void send_hil_gps(const ros::Time &stamp,
				uint8_t fix_type,
				int32_t lat, int32_t lon, int32_t alt,
				uint16_t eph, uint16_t epv, uint16_t vel,
				int16_t vn, int16_t ve, int16_t vd,
				uint16_t cog,
				uint8_t satellites_visible) {
		set_hil_gps(stamp.toNSec() / 1000,
					fix_type,
					lat, lon, alt,
					eph, epv, vel,
					vn, ve, vd,
					cog,
					satellites_visible);
	}

	/**
	 * @brief Send hil_sensor to FCU.
	 */
	void send_hil_sensor(const ros::Time &stamp,
				float xacc, float yacc, float zacc,
				float xgyro, float ygyro, float zgyro,
				float xmag, float ymag, float zmag,
				float abs_pressure, float diff_pressure, float pressure_alt,
				float temperature,
				uint32_t fields_updated) {
		set_hil_sensor(stamp.toNSec() / 1000,
					xacc, yacc, zacc,
					xgyro, ygyro, zgyro,
					xmag, ymag, zmag,
					abs_pressure, diff_pressure, pressure_alt,
					temperature,
					fields_updated);
	}

	/* -*- callbacks -*- */

	void state_quat_cb(const mavros_msgs::HilStateQuaternion::ConstPtr &req) {
		send_hil_state_quaternion(req->header.stamp,
					req->imu.orientation.w, req->imu.orientation.x, req->imu.orientation.y, req->imu.orientation.z,
					req->imu.angular_velocity.x,req->imu.angular_velocity.y, req->imu.angular_velocity.z,
					req->fix.latitude, req->fix.longitude, req->fix.altitude,
					req->linear_velocity.x, req->linear_velocity.y, req->linear_velocity.z,
					req->ind_airspeed,
					req->true_airspeed,
					req->imu.linear_acceleration.x, req->imu.linear_acceleration.y, req->imu.linear_acceleration.z);
	}

	void gps_cb(const mavros_msgs::HilGPS::ConstPtr &req) {
		send_hil_gps(req->header.stamp,
					req->fix_type,
					req->fix.latitude, req->fix.longitude, req->fix.altitude,
					req->eph, req->epv, req->vel,
					req->vn, req->ve, req->vd,
					req->cog,
					req->satellites_visible);
	}

	void sensor_cb(const mavros_msgs::HilSensor::ConstPtr &req) {
		// Throttle incoming messages to 40hz
		if ((ros::Time::now() - last_time) < period) {
			return;
		}
		last_time = ros::Time::now();

		send_hil_sensor(req->header.stamp,
					req->acc.x, req->acc.y, req->acc.z,
					req->gyro.x, req->gyro.y, req->gyro.z,
					req->mag.x, req->mag.y, req->mag.z,
					req->abs_pressure.fluid_pressure, req->diff_pressure.fluid_pressure, req->pressure_alt.fluid_pressure,
					req->temperature.temperature,
					req->fields_updated);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HilPlugin, mavros::plugin::PluginBase)

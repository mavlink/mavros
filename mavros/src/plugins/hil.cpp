/**
 * @brief Hil plugin
 * @file hil.cpp
 * @author Mohamed Abdelkader <mohamedashraf123@gmail.com>
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Pavel Vechersky <pvechersky@student.ethz.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016,2017 Mohamed Abdelkader, Nuno Marques, Pavel Vechersky, Beat Küng.
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
#include <mavros_msgs/OpticalFlowRad.h>
#include <mavros_msgs/RCIn.h>

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
		last_time_controls = ros::Time(0.0);
		last_time_sensor = ros::Time(0.0);
		period = ros::Duration(0.025);	// 40hz

		hil_state_quaternion_sub = hil_nh.subscribe("state", 10, &HilPlugin::state_quat_cb, this);
		hil_gps_sub = hil_nh.subscribe("gps", 10, &HilPlugin::gps_cb, this);
		hil_sensor_sub = hil_nh.subscribe("imu_ned", 10, &HilPlugin::sensor_cb, this);
		hil_flow_sub = hil_nh.subscribe("optical_flow", 10, &HilPlugin::optical_flow_cb, this);
		hil_rcin_sub = hil_nh.subscribe("rc_inputs", 10, &HilPlugin::rcin_raw_cb, this);

		hil_controls_pub = hil_nh.advertise<mavros_msgs::HilControls>("controls", 10);
		hil_actuator_controls_pub = hil_nh.advertise<mavros_msgs::HilActuatorControls>("actuator_controls", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&HilPlugin::handle_hil_controls),
			       make_handler(&HilPlugin::handle_hil_actuator_controls),
		};
	}

private:
	using lock_guard = std::lock_guard<std::mutex>;
	std::mutex mutex;
	ros::NodeHandle hil_nh;

	ros::Publisher hil_controls_pub;
	ros::Publisher hil_actuator_controls_pub;

	ros::Subscriber hil_state_quaternion_sub;
	ros::Subscriber hil_gps_sub;
	ros::Subscriber hil_sensor_sub;
	ros::Subscriber hil_flow_sub;
	ros::Subscriber hil_rcin_sub;

	ros::Time last_time_controls;
	ros::Time last_time_sensor;
	ros::Duration period;

	/* -*- rx handlers -*- */

	void handle_hil_controls(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HIL_CONTROLS &hil_controls) {
		// Throttle incoming messages to 40hz
		if ((ros::Time::now() - last_time_controls) < period) {
			return;
		}
		last_time_controls = ros::Time::now();

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
		hil_controls_msg->aux1 = hil_controls.aux1;
		hil_controls_msg->aux2 = hil_controls.aux2;
		hil_controls_msg->aux3 = hil_controls.aux3;
		hil_controls_msg->aux4 = hil_controls.aux4;
		hil_controls_msg->mode = hil_controls.mode;
		hil_controls_msg->nav_mode = hil_controls.nav_mode;
		// [[[end]]] (checksum: a2c87ee8f36e7a32b08be5e0fe665b5a)

		hil_controls_pub.publish(hil_controls_msg);
	}

	void handle_hil_actuator_controls(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HIL_ACTUATOR_CONTROLS &hil_actuator_controls) {
		auto hil_actuator_controls_msg = boost::make_shared<mavros_msgs::HilActuatorControls>();

		hil_actuator_controls_msg->header.stamp = m_uas->synchronise_stamp(hil_actuator_controls.time_usec);
		const auto &arr = hil_actuator_controls.controls;
		std::copy(arr.cbegin(), arr.cend(), hil_actuator_controls_msg->controls.begin());
		hil_actuator_controls_msg->mode = hil_actuator_controls.mode;
		hil_actuator_controls_msg->flags = hil_actuator_controls.flags;

		hil_actuator_controls_pub.publish(hil_actuator_controls_msg);
	}

	/* -*- callbacks / low level send -*- */

	/**
	 * @brief Send hil_state_quaternion to FCU.
	 * Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_STATE_QUATERNION
	 */
	void state_quat_cb(const mavros_msgs::HilStateQuaternion::ConstPtr &req) {
		mavlink::common::msg::HIL_STATE_QUATERNION state_quat;

		state_quat.time_usec = req->header.stamp.toNSec() / 1000;
		Eigen::Quaterniond q(req->imu.orientation.w,
					req->imu.orientation.x,
					req->imu.orientation.y,
					req->imu.orientation.z);
		ftf::quaternion_to_mavlink(q, state_quat.attitude_quaternion);
		state_quat.lat = req->geo.altitude * 1E7;
		state_quat.lon = req->geo.longitude * 1E7;
		// @warning geographic_msgs/GeoPoint.msg uses WGS 84 reference ellipsoid
		// @TODO: Convert altitude to AMSL to be received by the FCU
		// related to issue #529
		state_quat.alt = req->geo.altitude * 1E3;
		state_quat.ind_airspeed = req->ind_airspeed;
		state_quat.true_airspeed = req->true_airspeed;
		// [[[cog:
		// for a, b in zip(('rollspeed', 'pitchspeed', 'yawspeed'), "xyz"):
		//     cog.outl("state_quat.%s = req->imu.angular_velocity.%s;" % (a, b))
		// for f in "xyz":
		//     cog.outl("state_quat.v%s = req->linear_velocity.%s;" % (f, f))
		// for f in "xyz":
		//     cog.outl("state_quat.%sacc = req->imu.linear_acceleration.%s;" % (f, f))
		// ]]]
		state_quat.rollspeed = req->imu.angular_velocity.x;
		state_quat.pitchspeed = req->imu.angular_velocity.y;
		state_quat.yawspeed = req->imu.angular_velocity.z;
		state_quat.vx = req->linear_velocity.x;
		state_quat.vy = req->linear_velocity.y;
		state_quat.vz = req->linear_velocity.z;
		state_quat.xacc = req->imu.linear_acceleration.x;
		state_quat.yacc = req->imu.linear_acceleration.y;
		state_quat.zacc = req->imu.linear_acceleration.z;
		// [[[end]]] (checksum: ba223db81bec76579c3789d482d9721f)

		UAS_FCU(m_uas)->send_message_ignore_drop(state_quat);
	}

	/**
	 * @brief Send hil_gps to FCU.
	 * Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_GPS
	 */
	void gps_cb(const mavros_msgs::HilGPS::ConstPtr &req) {
		mavlink::common::msg::HIL_GPS gps;

		gps.time_usec = req->header.stamp.toNSec() / 1000;
		gps.fix_type = req->fix_type;
		gps.lat = req->geo.latitude * 1E7;
		gps.lon = req->geo.longitude * 1E7;
		// @warning geographic_msgs/GeoPoint.msg uses WGS 84 reference ellipsoid
		// @TODO: Convert altitude to AMSL to be received by the FCU
		// related to issue #529
		gps.alt = req->geo.altitude * 1E3;
		// [[[cog:
		// for f in ('eph', 'epv', 'vel', 'vn', 've', 'vd', 'cog', 'satellites_visible'):
		//     cog.outl("gps.%s = req->%s;" % (f, f))
		// ]]]
		gps.eph = req->eph;
		gps.epv = req->epv;
		gps.vel = req->vel;
		gps.vn = req->vn;
		gps.ve = req->ve;
		gps.vd = req->vd;
		gps.cog = req->cog;
		gps.satellites_visible = req->satellites_visible;
		// [[[end]]] (checksum: 8ed36ad860cde22e980f1d7ddcd1f6d9)

		UAS_FCU(m_uas)->send_message_ignore_drop(gps);
	}

	/**
	 * @brief Send hil_sensor to FCU.
	 * Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_SENSOR
	 */
	void sensor_cb(const mavros_msgs::HilSensor::ConstPtr &req) {
		// Throttle incoming messages to 40hz
		if ((ros::Time::now() - last_time_sensor) < period) {
			return;
		}
		last_time_sensor = ros::Time::now();

		mavlink::common::msg::HIL_SENSOR sensor;

		sensor.time_usec = req->header.stamp.toNSec() / 1000;
		// [[[cog:
		// for a in ('acc', 'gyro', 'mag'):
		//     for b in "xyz":
		//         cog.outl("sensor.{b}{a} = req->{a}.{b};".format(**locals()))
		// for f in ('abs_pressure', 'diff_pressure', 'pressure_alt'):
		//     cog.outl("sensor.%s = req->%s.fluid_pressure;" % (f, f))
		// ]]]
		sensor.xacc = req->acc.x;
		sensor.yacc = req->acc.y;
		sensor.zacc = req->acc.z;
		sensor.xgyro = req->gyro.x;
		sensor.ygyro = req->gyro.y;
		sensor.zgyro = req->gyro.z;
		sensor.xmag = req->mag.x;
		sensor.ymag = req->mag.y;
		sensor.zmag = req->mag.z;
		sensor.abs_pressure = req->abs_pressure.fluid_pressure;
		sensor.diff_pressure = req->diff_pressure.fluid_pressure;
		sensor.pressure_alt = req->pressure_alt.fluid_pressure;
		// [[[end]]] (checksum: c0fd710f77b66232ba2b645bf2c467c6)
		sensor.temperature = req->temperature.temperature;
		sensor.fields_updated = req->fields_updated;

		UAS_FCU(m_uas)->send_message_ignore_drop(sensor);
	}

	/**
	 * @brief Send simulated optical flow to FCU.
	 * Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_OPTICAL_FLOW
	 */
	void optical_flow_cb(const mavros_msgs::OpticalFlowRad::ConstPtr &req) {
		mavlink::common::msg::HIL_OPTICAL_FLOW of;

		auto int_xy = ftf::transform_frame_baselink_aircraft(
					Eigen::Vector3d(
						req->integrated_x,
						req->integrated_y,
						0.0));
		auto int_gyro = ftf::transform_frame_baselink_aircraft(
					Eigen::Vector3d(
						req->integrated_xgyro,
						req->integrated_ygyro,
						req->integrated_zgyro));

		of.time_usec = req->header.stamp.toNSec() / 1000;
		of.sensor_id = 0;
		of.integration_time_us = req->integration_time_us;
		// [[[cog:
		// for f in "xy":
		//     cog.outl("of.integrated_%s = int_xy.%s();" % (f, f))
		// for f in "xyz":
		//     cog.outl("of.integrated_%sgyro = int_gyro.%s();" % (f, f))
		// for f in ('time_delta_distance_us', 'distance', 'quality '):
		//     cog.outl("of.%s = req->%s;" % (f, f))
		// ]]]
		of.integrated_x = int_xy.x();
		of.integrated_y = int_xy.y();
		of.integrated_xgyro = int_gyro.x();
		of.integrated_ygyro = int_gyro.y();
		of.integrated_zgyro = int_gyro.z();
		of.time_delta_distance_us = req->time_delta_distance_us;
		of.distance = req->distance;
		of.quality  = req->quality;
		// [[[end]]] (checksum: 7c67e6b05dad4db31ccf2872ab34fa69)
		of.temperature = req->temperature * 100.0f;	// in centi-degrees celsius

		UAS_FCU(m_uas)->send_message_ignore_drop(of);
	}

	/**
	 * @brief Send simulated received RAW values of the RC channels to the FCU.
	 * Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_RC_INPUTS_RAW
	 */
	void rcin_raw_cb(const mavros_msgs::RCIn::ConstPtr &req) {
		mavlink::common::msg::HIL_RC_INPUTS_RAW rcin;

		constexpr size_t MAX_CHANCNT = 12;
		lock_guard lock(mutex);
		size_t channels_count = req->channels.size();

		if (channels_count > MAX_CHANCNT) {
			ROS_WARN_THROTTLE_NAMED(60, "hil_rc",
						"FCU receives %lu RC channels, but HIL_RC_INPUTS_RAW can store %zu",
						req->channels.size(), MAX_CHANCNT);

			channels_count = MAX_CHANCNT;
		}

		rcin.time_usec = req->header.stamp.toNSec() / 100000;
		// switch works as start point selector.
		switch (channels_count) {
		// [[[cog:
		// for i in range(12, 0, -1):
		//     cog.outl("case %2d: rcin.chan%d_raw \t= req->channels[%2d];" % (i, i, i-1))
		// ]]]
		case 12: rcin.chan12_raw	= req->channels[11];
		case 11: rcin.chan11_raw	= req->channels[10];
		case 10: rcin.chan10_raw	= req->channels[ 9];
		case  9: rcin.chan9_raw		= req->channels[ 8];
		case  8: rcin.chan8_raw		= req->channels[ 7];
		case  7: rcin.chan7_raw		= req->channels[ 6];
		case  6: rcin.chan6_raw		= req->channels[ 5];
		case  5: rcin.chan5_raw		= req->channels[ 4];
		case  4: rcin.chan4_raw		= req->channels[ 3];
		case  3: rcin.chan3_raw		= req->channels[ 2];
		case  2: rcin.chan2_raw		= req->channels[ 1];
		case  1: rcin.chan1_raw		= req->channels[ 0];
		// [[[end]]] (checksum: 02bafd8c28c3cf65a383aa60f15e5b00)
		case  0: break;
		}

		UAS_FCU(m_uas)->send_message_ignore_drop(rcin);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HilPlugin, mavros::plugin::PluginBase)

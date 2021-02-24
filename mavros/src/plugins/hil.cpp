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
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/HilControls.h>
#include <mavros_msgs/HilActuatorControls.h>
#include <mavros_msgs/HilStateQuaternion.h>
#include <mavros_msgs/HilGPS.h>
#include <mavros_msgs/HilSensor.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <mavros_msgs/RCIn.h>

namespace mavros {
namespace std_plugins {
//! Tesla to Gauss coeff
static constexpr double TESLA_TO_GAUSS = 1.0e4;
//! Pascal to millBar coeff
static constexpr double PASCAL_TO_MILLIBAR = 1.0e-2;

/**
 * @brief Hil plugin
 */
class HilPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	HilPlugin() : PluginBase(),
		hil_nh("~hil")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		hil_state_quaternion_sub = hil_nh.subscribe("state", 10, &HilPlugin::state_quat_cb, this);
		hil_gps_sub = hil_nh.subscribe("gps", 10, &HilPlugin::gps_cb, this);
		hil_sensor_sub = hil_nh.subscribe("imu_ned", 10, &HilPlugin::sensor_cb, this);
		hil_flow_sub = hil_nh.subscribe("optical_flow", 10, &HilPlugin::optical_flow_cb, this);
		hil_rcin_sub = hil_nh.subscribe("rc_inputs", 10, &HilPlugin::rcin_raw_cb, this);

		hil_controls_pub = hil_nh.advertise<mavros_msgs::HilControls>("controls", 10);
		hil_actuator_controls_pub = hil_nh.advertise<mavros_msgs::HilActuatorControls>("actuator_controls", 10);
	}

	Subscriptions get_subscriptions() override
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

	ros::Subscriber hil_state_quaternion_sub;
	ros::Subscriber hil_gps_sub;
	ros::Subscriber hil_sensor_sub;
	ros::Subscriber hil_flow_sub;
	ros::Subscriber hil_rcin_sub;

	Eigen::Quaterniond enu_orientation;

	/* -*- rx handlers -*- */

	void handle_hil_controls(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HIL_CONTROLS &hil_controls) {
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
	 * Message specification: @p https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION
	 */
	void state_quat_cb(const mavros_msgs::HilStateQuaternion::ConstPtr &req) {
		mavlink::common::msg::HIL_STATE_QUATERNION state_quat = {};

		state_quat.time_usec = req->header.stamp.toNSec() / 1000;
		auto q = ftf::transform_orientation_baselink_aircraft(
					ftf::transform_orientation_enu_ned(
						ftf::to_eigen(req->orientation)));
		ftf::quaternion_to_mavlink(q, state_quat.attitude_quaternion);
		state_quat.lat = req->geo.latitude * 1E7;
		state_quat.lon = req->geo.longitude * 1E7;
		// @warning geographic_msgs/GeoPoint.msg uses WGS 84 reference ellipsoid
		// @TODO: Convert altitude to AMSL to be received by the FCU
		// related to issue #529
		state_quat.alt = req->geo.altitude * 1E3;
		state_quat.ind_airspeed = req->ind_airspeed * 1E2;
		state_quat.true_airspeed = req->true_airspeed * 1E2;
		// WRT world frame
		auto ang_vel = ftf::transform_frame_enu_ned(
				ftf::transform_frame_baselink_aircraft(
					ftf::to_eigen(req->angular_velocity)));
		auto lin_vel = ftf::transform_frame_enu_ned<Eigen::Vector3d>(
				ftf::to_eigen(req->linear_velocity)) * 1E2;
		// linear acceleration - WRT world frame
		auto lin_acc = ftf::transform_frame_baselink_aircraft(
				ftf::to_eigen(req->linear_acceleration));

		// [[[cog:
		// for a, b in zip(('rollspeed', 'pitchspeed', 'yawspeed'), "xyz"):
		//     cog.outl("state_quat.%s = ang_vel.%s();" % (a, b))
		// for f in "xyz":
		//     cog.outl("state_quat.v%s = lin_vel.%s();" % (f, f))
		// for f in "xyz":
		//     cog.outl("state_quat.%sacc = lin_acc.%s();" % (f, f))
		// ]]]
		state_quat.rollspeed = ang_vel.x();
		state_quat.pitchspeed = ang_vel.y();
		state_quat.yawspeed = ang_vel.z();
		state_quat.vx = lin_vel.x();
		state_quat.vy = lin_vel.y();
		state_quat.vz = lin_vel.z();
		state_quat.xacc = lin_acc.x();
		state_quat.yacc = lin_acc.y();
		state_quat.zacc = lin_acc.z();
		// [[[end]]] (checksum: a29598b834ac1ec32ede01595aa5b3ac)

		UAS_FCU(m_uas)->send_message_ignore_drop(state_quat);
	}

	/**
	 * @brief Send hil_gps to FCU.
	 * Message specification: @p https://mavlink.io/en/messages/common.html#HIL_GPS
	 */
	void gps_cb(const mavros_msgs::HilGPS::ConstPtr &req) {
		mavlink::common::msg::HIL_GPS gps = {};

		gps.time_usec = req->header.stamp.toNSec() / 1000;
		gps.fix_type = req->fix_type;
		gps.lat = req->geo.latitude * 1E7;
		gps.lon = req->geo.longitude * 1E7;
		// @warning geographic_msgs/GeoPoint.msg uses WGS 84 reference ellipsoid
		// @TODO: Convert altitude to AMSL to be received by the FCU
		// related to issue #529
		gps.alt = req->geo.altitude * 1E3;
		// [[[cog:
		// for f in (
		//     'eph', 'epv', 'vel', 'vn', 've', 'vd', 'cog'):
		//     cog.outl("gps.%s = req->%s * 1E2;" % (f, f))
		// ]]]
		gps.eph = req->eph * 1E2;
		gps.epv = req->epv * 1E2;
		gps.vel = req->vel * 1E2;
		gps.vn = req->vn * 1E2;
		gps.ve = req->ve * 1E2;
		gps.vd = req->vd * 1E2;
		gps.cog = req->cog * 1E2;
		// [[[end]]] (checksum: a283bcc78f496cead2e9f893200d825d)
		gps.satellites_visible = req->satellites_visible;

		UAS_FCU(m_uas)->send_message_ignore_drop(gps);
	}

	/**
	 * @brief Send hil_sensor to FCU.
	 * Message specification: @p https://mavlink.io/en/messages/common.html#HIL_SENSOR
	 */
	void sensor_cb(const mavros_msgs::HilSensor::ConstPtr &req) {
		mavlink::common::msg::HIL_SENSOR sensor = {};

		sensor.time_usec = req->header.stamp.toNSec() / 1000;
		// WRT world frame
		auto acc = ftf::transform_frame_baselink_aircraft(
				ftf::to_eigen(req->acc));
		auto gyro = ftf::transform_frame_baselink_aircraft(
				ftf::to_eigen(req->gyro));
		auto mag = ftf::transform_frame_baselink_aircraft<Eigen::Vector3d>(
				ftf::to_eigen(req->mag) * TESLA_TO_GAUSS);

		// [[[cog:
		// for a in ('acc', 'gyro', 'mag'):
		//     for b in "xyz":
		//         cog.outl("sensor.{b}{a} = {a}.{b}();".format(**locals()))
		// for f in (('abs_pressure', 'PASCAL_TO_MILLIBAR'),
		//           ('diff_pressure', 'PASCAL_TO_MILLIBAR'),
		//           'pressure_alt', 'temperature', 'fields_updated'):
		//           f1 = f if isinstance(f, str) else f[0]
		//           f2 = f if isinstance(f, str) else '{f[0]} * {f[1]}'.format(f=f)
		//           cog.outl("sensor.{f1} = req->{f2};".format(**locals()))
		// ]]]
		sensor.xacc = acc.x();
		sensor.yacc = acc.y();
		sensor.zacc = acc.z();
		sensor.xgyro = gyro.x();
		sensor.ygyro = gyro.y();
		sensor.zgyro = gyro.z();
		sensor.xmag = mag.x();
		sensor.ymag = mag.y();
		sensor.zmag = mag.z();
		sensor.abs_pressure = req->abs_pressure * PASCAL_TO_MILLIBAR;
		sensor.diff_pressure = req->diff_pressure * PASCAL_TO_MILLIBAR;
		sensor.pressure_alt = req->pressure_alt;
		sensor.temperature = req->temperature;
		sensor.fields_updated = req->fields_updated;
		// [[[end]]] (checksum: 316bef821ad6fc33d9726a1c8e8c5404)

		UAS_FCU(m_uas)->send_message_ignore_drop(sensor);
	}

	/**
	 * @brief Send simulated optical flow to FCU.
	 * Message specification: @p https://mavlink.io/en/messages/common.html#HIL_OPTICAL_FLOW
	 */
	void optical_flow_cb(const mavros_msgs::OpticalFlowRad::ConstPtr &req) {
		mavlink::common::msg::HIL_OPTICAL_FLOW of = {};

		auto int_xy = ftf::transform_frame_aircraft_baselink(
					Eigen::Vector3d(
						req->integrated_x,
						req->integrated_y,
						0.0));
		auto int_gyro = ftf::transform_frame_aircraft_baselink(
					Eigen::Vector3d(
						req->integrated_xgyro,
						req->integrated_ygyro,
						req->integrated_zgyro));

		of.time_usec = req->header.stamp.toNSec() / 1000;
		of.sensor_id = INT8_MAX;//while we don't find a better way of handling it
		of.integration_time_us = req->integration_time_us;
		// [[[cog:
		// for f in "xy":
		//     cog.outl("of.integrated_%s = int_xy.%s();" % (f, f))
		// for f in "xyz":
		//     cog.outl("of.integrated_%sgyro = int_gyro.%s();" % (f, f))
		// for f in ('time_delta_distance_us', 'distance', 'quality'):
		//     cog.outl("of.%s = req->%s;" % (f, f))
		// ]]]
		of.integrated_x = int_xy.x();
		of.integrated_y = int_xy.y();
		of.integrated_xgyro = int_gyro.x();
		of.integrated_ygyro = int_gyro.y();
		of.integrated_zgyro = int_gyro.z();
		of.time_delta_distance_us = req->time_delta_distance_us;
		of.distance = req->distance;
		of.quality = req->quality;
		// [[[end]]] (checksum: acbfae28f4f3bb8ca135423efaaa479e)
		of.temperature = req->temperature * 100.0f;	// in centi-degrees celsius

		UAS_FCU(m_uas)->send_message_ignore_drop(of);
	}

	/**
	 * @brief Send simulated received RAW values of the RC channels to the FCU.
	 * Message specification: @p https://mavlink.io/en/messages/common.html#HIL_RC_INPUTS_RAW
	 */
	void rcin_raw_cb(const mavros_msgs::RCIn::ConstPtr &req) {
		mavlink::common::msg::HIL_RC_INPUTS_RAW rcin {};

		constexpr size_t MAX_CHANCNT = 12;

		std::array<uint16_t, MAX_CHANCNT> channels;
		auto n = std::min(req->channels.size(), channels.size());
		std::copy(req->channels.begin(), req->channels.begin() + n, channels.begin());
		std::fill(channels.begin() + n, channels.end(), UINT16_MAX);

		rcin.time_usec = req->header.stamp.toNSec() / 100000;
		// [[[cog:
		// for i in range(1,13):
		//     cog.outl("rcin.chan%d_raw\t= channels[%2d];" % (i, i-1))
		// ]]]
		rcin.chan1_raw	= channels[ 0];
		rcin.chan2_raw	= channels[ 1];
		rcin.chan3_raw	= channels[ 2];
		rcin.chan4_raw	= channels[ 3];
		rcin.chan5_raw	= channels[ 4];
		rcin.chan6_raw	= channels[ 5];
		rcin.chan7_raw	= channels[ 6];
		rcin.chan8_raw	= channels[ 7];
		rcin.chan9_raw	= channels[ 8];
		rcin.chan10_raw	= channels[ 9];
		rcin.chan11_raw	= channels[10];
		rcin.chan12_raw	= channels[11];
		// [[[end]]] (checksum: 8d6860789d596dc39e81b351c3a50fcd)

		UAS_FCU(m_uas)->send_message_ignore_drop(rcin);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HilPlugin, mavros::plugin::PluginBase)

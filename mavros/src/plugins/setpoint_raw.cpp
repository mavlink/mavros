/**
 * @brief SetpointRAW plugin
 * @file setpoint_raw.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <tf/transform_datatypes.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

namespace mavplugin {
/**
 * @brief Setpoint RAW plugin
 *
 * Send position setpoints and publish current state (return loop).
 * User can decide what set of filed needed for operation via IGNORE bits.
 */
class SetpointRawPlugin : public MavRosPlugin,
	private SetPositionTargetLocalNEDMixin<SetpointRawPlugin> {
public:
	SetpointRawPlugin() :
		sp_nh("~setpoint_raw"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		bool tf_listen;

		uas = &uas_;

		ignore_rpyt_messages_ = false;
		if(!sp_nh.getParam("thrust_scaling_factor", thrust_scaling_)){
			ROS_FATAL("No thrust scaling factor found, DO NOT FLY");
			ignore_rpyt_messages_ = true;
		}
		if(!sp_nh.getParam("system_mass_kg", system_mass_kg_)){
			ROS_FATAL("No system mass found, DO NOT FLY");
			ignore_rpyt_messages_ = true;
		}
    if(!sp_nh.getParam("yaw_rate_scaling_factor", yaw_rate_scaling_)){
      ROS_FATAL("No yaw rate scaling factor found, DO NOT FLY");
      ignore_rpyt_messages_ = true;
    }

		local_sub = sp_nh.subscribe("local", 10, &SetpointRawPlugin::local_cb, this);
		global_sub = sp_nh.subscribe("global", 10, &SetpointRawPlugin::global_cb, this);
		attitude_sub = sp_nh.subscribe("attitude", 10, &SetpointRawPlugin::attitude_cb, this);
		rpyt_sub = sp_nh.subscribe("roll_pitch_yawrate_thrust", 10, &SetpointRawPlugin::rpyt_cb, this);
		target_local_pub = sp_nh.advertise<mavros_msgs::PositionTarget>("target_local", 10);
		target_global_pub = sp_nh.advertise<mavros_msgs::GlobalPositionTarget>("target_global", 10);
		target_attitude_pub = sp_nh.advertise<mavros_msgs::AttitudeTarget>("target_attitude", 10);
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, &SetpointRawPlugin::handle_position_target_local_ned),
			MESSAGE_HANDLER(MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT, &SetpointRawPlugin::handle_position_target_global_int),
			MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE_TARGET, &SetpointRawPlugin::handle_attitude_target),
		};
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber local_sub, global_sub, attitude_sub, rpyt_sub;
	ros::Publisher target_local_pub, target_global_pub, target_attitude_pub;
	double thrust_scaling_, system_mass_kg_, yaw_rate_scaling_;
	bool ignore_rpyt_messages_;

	/* -*- message handlers -*- */
	void handle_position_target_local_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_position_target_local_ned_t tgt;
		mavlink_msg_position_target_local_ned_decode(msg, &tgt);

		// Transform desired position,velocities,and accels from ENU to NED frame
		auto position = UAS::transform_frame_ned_enu(Eigen::Vector3d(tgt.x, tgt.y, tgt.z));
		auto velocity = UAS::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
		auto af = UAS::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
		float yaw = UAS::transform_frame_yaw_ned_enu(tgt.yaw);
		float yaw_rate = UAS::transform_frame_yaw_ned_enu(tgt.yaw_rate);

		auto target = boost::make_shared<mavros_msgs::PositionTarget>();

		target->header.stamp = uas->synchronise_stamp(tgt.time_boot_ms);
		target->coordinate_frame = tgt.coordinate_frame;
		target->type_mask = tgt.type_mask;
		tf::pointEigenToMsg(position, target->position);
		tf::vectorEigenToMsg(velocity, target->velocity);
		tf::vectorEigenToMsg(af, target->acceleration_or_force);
		target->yaw = yaw;
		target->yaw_rate = yaw_rate;

		target_local_pub.publish(target);
	}

	void handle_position_target_global_int(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_position_target_global_int_t tgt;
		mavlink_msg_position_target_global_int_decode(msg, &tgt);

		// Transform desired velocities from ENU to NED frame
		auto velocity = UAS::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
		auto af = UAS::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
		float yaw = UAS::transform_frame_yaw_ned_enu(tgt.yaw);
		float yaw_rate = UAS::transform_frame_yaw_ned_enu(tgt.yaw_rate);

		auto target = boost::make_shared<mavros_msgs::GlobalPositionTarget>();

		target->header.stamp = uas->synchronise_stamp(tgt.time_boot_ms);
		target->coordinate_frame = tgt.coordinate_frame;
		target->type_mask = tgt.type_mask;
		target->latitude = tgt.lat_int / 1e7;
		target->longitude = tgt.lon_int / 1e7;
		target->altitude = tgt.alt;
		tf::vectorEigenToMsg(velocity, target->velocity);
		tf::vectorEigenToMsg(af, target->acceleration_or_force);
		target->yaw = yaw;
		target->yaw_rate = yaw_rate;

		target_global_pub.publish(target);
	}

	void handle_attitude_target(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_attitude_target_t tgt;
		mavlink_msg_attitude_target_decode(msg, &tgt);

		// Transform orientation from baselink -> ENU
		// to aircraft -> NED
		auto orientation = UAS::transform_orientation_ned_enu(
						   UAS::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tgt.q[0], tgt.q[1], tgt.q[2], tgt.q[3])));

		auto body_rate = UAS::transform_frame_baselink_aircraft(Eigen::Vector3d(tgt.body_roll_rate, tgt.body_pitch_rate, tgt.body_yaw_rate));

		auto target = boost::make_shared<mavros_msgs::AttitudeTarget>();

		target->header.stamp = uas->synchronise_stamp(tgt.time_boot_ms);
		target->type_mask = tgt.type_mask;
		tf::quaternionEigenToMsg(orientation, target->orientation);
		tf::vectorEigenToMsg(body_rate, target->body_rate);
		target->thrust = tgt.thrust;

		target_attitude_pub.publish(target);
	}

	/* -*- low-level send -*- */
	//! Message specification: @p http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
	void set_position_target_global_int(uint32_t time_boot_ms, uint8_t coordinate_frame, uint8_t type_mask,
			int32_t lat_int, int32_t lon_int, float alt,
			Eigen::Vector3d &velocity,
			Eigen::Vector3d &af,
			float yaw, float yaw_rate) {
		mavlink_message_t msg;
		mavlink_msg_set_position_target_global_int_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_boot_ms,
				UAS_PACK_TGT(uas),
				coordinate_frame,
				type_mask,
				lat_int, lon_int, alt,
				velocity.x(), velocity.y(), velocity.z(),
				af.x(), af.y(), af.z(),
				yaw, yaw_rate);
		UAS_FCU(uas)->send_message(&msg);
	}

	//! Message sepecification: @p http://mavlink.org/messages/common#SET_ATTITIDE_TARGET
	void set_attitude_target(uint32_t time_boot_ms,
			uint8_t type_mask,
			Eigen::Quaterniond &orientation,
			Eigen::Vector3d &body_rate,
			float thrust) {
		float q[4];
		UAS::quaternion_to_mavlink(orientation, q);

		mavlink_message_t msg;
		mavlink_msg_set_attitude_target_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_boot_ms,
				UAS_PACK_TGT(uas),
				type_mask,
				q,
				body_rate.x(), body_rate.y(), body_rate.z(),
				thrust);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */

	void local_cb(const mavros_msgs::PositionTarget::ConstPtr &req) {
		Eigen::Vector3d position, velocity, af;
		float yaw, yaw_rate;

		tf::pointMsgToEigen(req->position, position);
		tf::vectorMsgToEigen(req->velocity, velocity);
		tf::vectorMsgToEigen(req->acceleration_or_force, af);

		// Transform frame ENU->NED
		position = UAS::transform_frame_enu_ned(position);
		velocity = UAS::transform_frame_enu_ned(velocity);
		af = UAS::transform_frame_enu_ned(af);
		yaw = UAS::transform_frame_yaw_enu_ned(req->yaw);
		yaw_rate = UAS::transform_frame_yaw_enu_ned(req->yaw_rate);

		set_position_target_local_ned(
				req->header.stamp.toNSec() / 1000000,
				req->coordinate_frame,
				req->type_mask,
				position.x(), position.y(), position.z(),
				velocity.x(), velocity.y(), velocity.z(),
				af.x(), af.y(), af.z(),
				yaw, yaw_rate);
	}

	void global_cb(const mavros_msgs::GlobalPositionTarget::ConstPtr &req) {
		Eigen::Vector3d velocity, af;
		float yaw, yaw_rate;

		tf::vectorMsgToEigen(req->velocity, velocity);
		tf::vectorMsgToEigen(req->acceleration_or_force, af);

		// Transform frame ENU->NED
		velocity = UAS::transform_frame_enu_ned(velocity);
		af = UAS::transform_frame_enu_ned(af);
		yaw = UAS::transform_frame_yaw_enu_ned(req->yaw);
		yaw_rate = UAS::transform_frame_yaw_enu_ned(req->yaw_rate);

		set_position_target_global_int(
				req->header.stamp.toNSec() / 1000000,
				req->coordinate_frame,
				req->type_mask,
				req->latitude * 1e7,
				req->longitude * 1e7,
				req->altitude,
				velocity, af,
				yaw, yaw_rate);
	}

	void attitude_cb(const mavros_msgs::AttitudeTarget::ConstPtr &req) {
		Eigen::Quaterniond desired_orientation;
		Eigen::Vector3d baselink_angular_rate;


		tf::quaternionMsgToEigen(req->orientation, desired_orientation);

		// Transform desired orientation to represent aircraft->NED,
		// MAVROS operates on orientation of base_link->ENU
		auto ned_desired_orientation = UAS::transform_orientation_enu_ned(
			UAS::transform_orientation_baselink_aircraft(desired_orientation));

		auto body_rate = UAS::transform_frame_baselink_aircraft(baselink_angular_rate);

		tf::vectorMsgToEigen(req->body_rate, body_rate);

		set_attitude_target(
				req->header.stamp.toNSec() / 1000000,
				req->type_mask,
				ned_desired_orientation,
				body_rate,
				req->thrust);
	}

	void rpyt_cb(const mav_msgs::RollPitchYawrateThrustConstPtr msg) {

		if(ignore_rpyt_messages_){
			ROS_FATAL("Recieved roll_pitch_yaw_thrust_rate message, but ignore_rpyt_messages_ is true: "
      "the most likely cause of this is a failure to specify the thrust_scaling_factor,	"
      "yaw_rate_scaling_factor or system_mass_kg parameters");
      return;
		}

		//the masks are much more limited than the docs would suggest so we don't use them
  	uint8_t type_mask = 0;

  	geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(
      msg->roll, msg->pitch, 0);

		double thrust = std::min(1.0, std::max(0.0, msg->thrust.z * thrust_scaling_ * system_mass_kg_));

		Eigen::Quaterniond desired_orientation;
		Eigen::Vector3d body_rate;

		tf::quaternionMsgToEigen(orientation, desired_orientation);

		// Transform desired orientation to represent aircraft->NED,
		// MAVROS operates on orientation of base_link->ENU
		auto ned_desired_orientation = UAS::transform_orientation_enu_ned(
			UAS::transform_orientation_baselink_aircraft(desired_orientation));

    body_rate.x() = 0;
    body_rate.y() = 0;
		body_rate.z() = -yaw_rate_scaling_*msg->yaw_rate;

		set_attitude_target(
				msg->header.stamp.toNSec() / 1000000,
				type_mask,
				ned_desired_orientation,
				body_rate,
				thrust);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointRawPlugin, mavplugin::MavRosPlugin)

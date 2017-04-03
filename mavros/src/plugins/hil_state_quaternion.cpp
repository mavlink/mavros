/**
 * @brief HilStateQuaternion plugin
 * @file hil_state_quaternion.cpp
 * @author Mohamed Abdelkader <mohamedashraf123@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 Mohamed Abdelkader.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/HilStateQuaternion.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief HIL State Quaternion plugin
 *
 * Send HIL_STATE_QUATERNION msg to FCU controller.
 */
class HilStateQuaternionPlugin : public plugin::PluginBase {
public:
	HilStateQuaternionPlugin() : PluginBase(),
		state_quat_nh("~hil_state_quaternion")

	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		hilStateQuaternion_sub = state_quat_nh.subscribe("hil_state", 10, &HilStateQuaternionPlugin::state_quat_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle state_quat_nh;

	ros::Subscriber hilStateQuaternion_sub;

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

      state_quat.time_usec= time_boot_us;

      state_quat.attitude_quaternion[0]=qw;
      state_quat.attitude_quaternion[1]=qx;
      state_quat.attitude_quaternion[2]=qy;
      state_quat.attitude_quaternion[3]=qz;

      state_quat.rollspeed=rollspeed;
      state_quat.pitchspeed=pitchspeed;
      state_quat.yawspeed=yawspeed;

      state_quat.lat=lat;
      state_quat.lon=lon;
      state_quat.alt=alt;

      state_quat.vx=vx;
      state_quat.vy=vy;
      state_quat.vz=vz;

      state_quat.ind_airspeed=ind_airspeed;
      state_quat.true_airspeed=true_airspeed;

      state_quat.xacc=xacc;
      state_quat.yacc=yacc;
      state_quat.zacc=zacc;

      UAS_FCU(m_uas)->send_message_ignore_drop(state_quat);
  }

	/* -*- mid-level helpers -*- */
	/**
	 * @brief Send hil_state_quaternion to FCU.
	 *
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
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HilStateQuaternionPlugin, mavros::plugin::PluginBase)

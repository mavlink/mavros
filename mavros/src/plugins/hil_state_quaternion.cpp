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
#include <mavros/hil_sensor_mixin.h>// TODO: add SetHilStateQuaternionMixin
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/HilStateQuaternion.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief HIL State Quaternion plugin
 *
 * Send HIL_STATE_QUATERNION msg to FCU controller.
 */
class HilStateQuaternionPlugin : public plugin::PluginBase,
    private plugin::SetHilStateQuaternionMixin<HilStateQuaternionPlugin> {
public:
	HilStateQuaternionPlugin() : PluginBase(),
		sp_nh("~hil_state_quaternion"),

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
	friend class SetHilStateQuaternionMixin;
        
	ros::NodeHandle state_quat_nh;

	ros::Subscriber hilStateQuaternion_sub;


	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send hil_state_quaternion to FCU.
	 *
	 */
	void send_hil_state_quaternion(const ros::Time &stamp,
                                   Eigen::Vector4d &quat,
                                   Eigen::Vector3d &angularspeed,
                                   Eigen::Vector3d &GPS,
                                   Eigen::Vector3d &groundspeed,
                                   uint16_t ind_airspeed,
                                   uint16_t true_airspeed,
                                   Eigen::Vector3d acceleration) {

		set_hil_state_quaternion(stamp.toNSec() / 1000,
					quat,
					angularspeed,
					GPS,
					groundspeed,
					ind_airspeed,
					true_airspeed,
                    acceleration);
	}

	/* -*- callbacks -*- */
        

		void state_quat_cb(const mavros_msgs::HilStateQuaternion::ConstPtr &req) {
            Eigen::Vector4d quat;
            Eigen::Vector3d angularspeed;
            Eigen::Vector3d GPS;
            Eigen::Vector3d groundspeed;
            uint16_t ind_airspeed;
            uint16_t true_airspeed;
            Eigen::Vector3d acceleration;
            
            tf::vectorMsgToEigen(req->quaternion_wxyz, quat);
            tf::vectorMsgToEigen(req->angularspeed, angularspeed);
            tf::vectorMsgToEigen(req->GPS, GPS);
            tf::vectorMsgToEigen(req->groundspeed, groundspeed);
            tf::vectorMsgToEigen(req->acceleration, acceleration);
            
            send_hil_sensor(req->header.stamp,
                            quat,
                            angularspeed,
                            GPS,
                            groundspeed,
                            req->ind_airspeed,
                            req->true_airspeed,
                            acceleration);
        }
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HilStateQuaternionPlugin, mavros::plugin::PluginBase)

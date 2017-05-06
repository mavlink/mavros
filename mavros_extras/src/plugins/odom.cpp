/**
 * @brief Odometry plugin
 * @file odom.cpp
 * @author James Goppert <james.goppert8@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 James Goppert
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Odometry.h>

namespace mavros {
namespace extra_plugins{
/**
 * @brief Odometry plugin
 *
 * Send odometry info
 * to FCU position and attitude estimators.
 *
 */
class OdometryPlugin : public plugin::PluginBase {
public:
	OdometryPlugin() : PluginBase(),
		_nh("~odometry")
	{
	}

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// tf params
		_odom_sub = _nh.subscribe("odom", 10, &OdometryPlugin::odom_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle _nh;
	ros::Subscriber _odom_sub;

	/* -*- callbacks -*- */

	void odom_cb(const nav_msgs::Odometry::ConstPtr &odom)
	{
		//ROS_INFO("odometry callback");
		uint64_t stamp = odom->header.stamp.toNSec() / 1e3;

		// send local position
		mavlink::common::msg::LOCAL_POSITION_NED_COV lpos{};
		lpos.time_usec = stamp;
		// NED to ENU
		// TODO check these are in the correct frame
		lpos.x = odom->pose.pose.position.y;
		lpos.y = odom->pose.pose.position.x;
		lpos.z = -odom->pose.pose.position.z;
		lpos.vx = odom->twist.twist.linear.y;
		lpos.vy = odom->twist.twist.linear.x;
		lpos.vz = -odom->twist.twist.linear.z;
		lpos.ax = 0;
		lpos.ay = 0;
		lpos.az = 0;
		size_t i = 0;
		for (int row=0; row < 6; row++) {
			for (int col=row; col < 6; col++) {
				//ROS_INFO("row: %d, col:%d, i:%d", row, col, i);
				lpos.covariance[i] = odom->pose.covariance[row*6 + col];
				i += 1;
			}
		}
		UAS_FCU(m_uas)->send_message_ignore_drop(lpos);

		// send attitude
		mavlink::common::msg::ATTITUDE_QUATERNION_COV att;
		// TODO check these are in the correct frame
		att.rollspeed =  odom->twist.twist.angular.y;
		att.pitchspeed =  odom->twist.twist.angular.x;
		att.yawspeed =  -odom->twist.twist.angular.z;

		att.time_usec = stamp;
		att.q[0] = odom->pose.pose.orientation.w;
		att.q[1] = odom->pose.pose.orientation.x;
		att.q[2] = odom->pose.pose.orientation.y;
		att.q[3] = odom->pose.pose.orientation.z;
		for (size_t i=0; i < 9; i++) {
			att.covariance[i] = odom->pose.covariance[i];
		}
		UAS_FCU(m_uas)->send_message_ignore_drop(att);

	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OdometryPlugin, mavros::plugin::PluginBase)

/**
 * @brief VisionSpeedEstimate plugin
 * @file vision_speed.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace mavros {
namespace extra_plugins{
/**
 * @brief Vision speed estimate plugin
 *
 * Send velocity estimation from various vision estimators
 * to FCU position and attitude estimators.
 *
 */
class VisionSpeedEstimatePlugin : public plugin::PluginBase {
public:
	VisionSpeedEstimatePlugin() : PluginBase(),
		sp_nh("~vision_speed")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		bool listen_twist;

		sp_nh.param("listen_twist", listen_twist, false);

		if (listen_twist)
			vision_vel_sub = sp_nh.subscribe("speed_twist", 10, &VisionSpeedEstimatePlugin::vel_twist_cb, this);
		else
			vision_vel_sub = sp_nh.subscribe("speed_vector", 10, &VisionSpeedEstimatePlugin::vel_speed_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle sp_nh;

	ros::Subscriber vision_vel_sub;

	/* -*- low-level send -*- */

	void vision_speed_estimate(uint64_t usec, Eigen::Vector3d &v)
	{
		mavlink::common::msg::VISION_SPEED_ESTIMATE vs{};

		vs.usec = usec;

		// [[[cog:
		// for f in "xyz":
		//     cog.outl("vs.%s = v.%s();" % (f, f))
		// ]]]
		vs.x = v.x();
		vs.y = v.y();
		vs.z = v.z();
		// [[[end]]] (checksum: aee3cc9a73a2e736b7bc6c83ea93abdb)

		UAS_FCU(m_uas)->send_message_ignore_drop(vs);
	}

	/**
	 * @todo Suggest modification on PX4 firmware to MAVLINK VISION_SPEED_ESTIMATE
	 * msg name, which should be called instead VISION_VELOCITY_ESTIMATE
	 */

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send vision speed estimate to FCU velocity controller
	 */
	void send_vision_speed(const geometry_msgs::Vector3 &vel_enu, const ros::Time &stamp)
	{
		Eigen::Vector3d vel_;
		tf::vectorMsgToEigen(vel_enu, vel_);
		//Transform from ENU to NED frame
		auto vel = ftf::transform_frame_enu_ned(vel_);

		vision_speed_estimate(stamp.toNSec() / 1000, vel);
	}

	/* -*- callbacks -*- */

	void vel_twist_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
		send_vision_speed(req->twist.linear, req->header.stamp);
	}

	void vel_speed_cb(const geometry_msgs::Vector3Stamped::ConstPtr &req) {
		send_vision_speed(req->vector, req->header.stamp);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VisionSpeedEstimatePlugin, mavros::plugin::PluginBase)

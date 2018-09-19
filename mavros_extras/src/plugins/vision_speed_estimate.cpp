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
 * Copyright 2014, 2018 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <geometry_msgs/Vector3Stamped.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Vision speed estimate plugin
 *
 * Send velocity estimation from various vision estimators
 * to FCU position and attitude estimators.
 */
class VisionSpeedEstimatePlugin : public plugin::PluginBase {
public:
	VisionSpeedEstimatePlugin() : PluginBase(),
		sp_nh("~vision_speed"),
		listen_twist(true),
		twist_cov(true)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		sp_nh.param("listen_twist", listen_twist, true);
		sp_nh.param("twist_cov", twist_cov, true);

		if (listen_twist) {
			if (twist_cov)
				vision_twist_cov_sub = sp_nh.subscribe("speed_twist_cov", 10, &VisionSpeedEstimatePlugin::twist_cov_cb, this);
			else
				vision_twist_sub = sp_nh.subscribe("speed_twist", 10, &VisionSpeedEstimatePlugin::twist_cb, this);
		}
		else
			vision_vector_sub = sp_nh.subscribe("speed_vector", 10, &VisionSpeedEstimatePlugin::vector_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle sp_nh;

	bool listen_twist;			//!< If True, listen to Twist data topics
	bool twist_cov;				//!< If True, listen to TwistWithCovariance data topic

	ros::Subscriber vision_twist_sub;	//!< Subscriber to geometry_msgs/TwistStamped msgs
	ros::Subscriber vision_twist_cov_sub;	//!< Subscriber to geometry_msgs/TwistWithCovarianceStamped msgs
	ros::Subscriber vision_vector_sub;	//!< Subscriber to geometry_msgs/Vector3Stamped msgs

	/* -*- low-level send -*- */
	/**
	 * @brief Send vision speed estimate on local NED frame to the FCU.
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
	 * @param usec	Timestamp (microseconds, synced to UNIX time or since system boot) (us)
	 * @param v	Velocity/speed vector in the local NED frame (meters)
	 * @param cov	Linear velocity covariance matrix (local NED frame)
	 */
	void send_vision_speed_estimate(const uint64_t usec, const Eigen::Vector3d &v, const ftf::Covariance3d &cov)
	{
		mavlink::common::msg::VISION_SPEED_ESTIMATE vs {};

		vs.usec = usec;

		// [[[cog:
		// for f in "xyz":
		//     cog.outl("vs.%s = v.%s();" % (f, f))
		// ]]]
		vs.x = v.x();
		vs.y = v.y();
		vs.z = v.z();
		// [[[end]]] (checksum: aee3cc9a73a2e736b7bc6c83ea93abdb)

		ftf::covariance_to_mavlink(cov, vs.covariance);

		UAS_FCU(m_uas)->send_message_ignore_drop(vs);
	}

	/* -*- mid-level helpers -*- */
	/**
	 * @brief Convert vector and covariance from local ENU to local NED frame
	 *
	 * @param stamp		ROS timestamp of the message
	 * @param vel_enu	Velocity/speed vector in the ENU frame
	 * @param cov_enu	Linear velocity/speed in the ENU frame
	 */
	void convert_vision_speed(const ros::Time &stamp, const Eigen::Vector3d &vel_enu, const ftf::Covariance3d &cov_enu)
	{
		// Send transformed data from local ENU to NED frame
		send_vision_speed_estimate(stamp.toNSec() / 1000,
					ftf::transform_frame_enu_ned(vel_enu),
					ftf::transform_frame_enu_ned(cov_enu));
	}

	/* -*- callbacks -*- */
	/**
	 * @brief Callback to geometry_msgs/TwistStamped msgs
	 *
	 * @param req	received geometry_msgs/TwistStamped msg
	 */
	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
		ftf::Covariance3d cov {};	// zero initialized

		convert_vision_speed(req->header.stamp, ftf::to_eigen(req->twist.linear), cov);
	}

	/**
	 * @brief Callback to geometry_msgs/TwistWithCovarianceStamped msgs
	 *
	 * @param req	received geometry_msgs/TwistWithCovarianceStamped msg
	 */
	void twist_cov_cb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &req) {
		ftf::Covariance3d cov3d {};	// zero initialized

		ftf::EigenMapCovariance3d cov3d_map(cov3d.data());
		ftf::EigenMapConstCovariance6d cov6d_map(req->twist.covariance.data());

		// only the linear velocity will be sent
		cov3d_map = cov6d_map.block<3, 3>(0, 0);

		convert_vision_speed(req->header.stamp, ftf::to_eigen(req->twist.twist.linear), cov3d);
	}

	/**
	 * @brief Callback to geometry_msgs/Vector3Stamped msgs
	 *
	 * @param req	received geometry_msgs/Vector3Stamped msg
	 */
	void vector_cb(const geometry_msgs::Vector3Stamped::ConstPtr &req) {
		ftf::Covariance3d cov {};	// zero initialized

		convert_vision_speed(req->header.stamp, ftf::to_eigen(req->vector), cov);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VisionSpeedEstimatePlugin, mavros::plugin::PluginBase)

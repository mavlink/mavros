/**
 * @brief Guided target plugin
 * @file guided_target.cpp
 * @author Randy Mackay <rmackay9@yahoo.com> , Sanket Sharma <sharma.sanket272@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2022 Sanket Sharma, Randy Mackay.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/SetMavFrame.h>
#include <geographic_msgs/GeoPoseStamped.h>

#include <GeographicLib/Geocentric.hpp>

#include <mavros_msgs/PositionTarget.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/GlobalPositionTarget.h>

namespace mavros {
namespace extra_plugins {
	
/**
 * @brief guided target plugin
 *
 * Send and receive setpoint positions from FCU controller.
 */
class GuidedTargetPlugin : public plugin::PluginBase,
	private plugin::SetPositionTargetLocalNEDMixin<GuidedTargetPlugin>,
	private plugin::SetPositionTargetGlobalIntMixin<GuidedTargetPlugin>,
	private plugin::TF2ListenerMixin<GuidedTargetPlugin> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	GuidedTargetPlugin() : PluginBase(),
		sp_nh("~guided_target"),
		spg_nh("~"),
		tf_listen(false),
		tf_rate(50.0),
		is_map_init(false)
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// tf params
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "target_position");
		sp_nh.param("tf/rate_limit", tf_rate, 50.0);

		// Publish targets received from FCU
		setpointg_pub = spg_nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);


		// Subscriber for global origin (aka map origin).
		gp_origin_sub = spg_nh.subscribe("global_position/gp_origin", 10, &GuidedTargetPlugin::gp_origin_cb, this);	
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&GuidedTargetPlugin::handle_position_target_global_int)
		}; 
	}

private:


	ros::NodeHandle sp_nh;
	ros::NodeHandle spg_nh;		//!< to get local position and gps coord which are not under sp_h()

	ros::Subscriber gp_origin_sub; //!< global origin LLA

	ros::Publisher setpointg_pub; //!< global position target from FCU

	/* Stores current gps state. */
	//sensor_msgs::NavSatFix current_gps_msg;
	Eigen::Vector3d current_gps;		//!< geodetic coordinates LLA
	Eigen::Vector3d current_local_pos;	//!< Current local position in ENU

	Eigen::Vector3d map_origin {};	//!< oigin of map frame [lla]
	Eigen::Vector3d ecef_origin {};	//!< geocentric origin [m]

	uint32_t old_gps_stamp = 0;		//!< old time gps time stamp in [ms], to check if new gps msg is received

	std::string tf_frame_id;
	std::string tf_child_frame_id;

	bool tf_listen;
	double tf_rate;
	bool is_map_init;

	double arr[2] = {0, 0};

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send setpoint to FCU position controller.
	 *
	 * @warning Send only XYZ, Yaw. ENU frame.
	 */
	

	/**
	 * global origin in LLA
	 */
	void gp_origin_cb(const geographic_msgs::GeoPointStamped::ConstPtr &msg)
	{
		ecef_origin = {msg->position.latitude, msg->position.longitude, msg->position.altitude};
		/**
		 * @brief Conversion from ECEF (Earth-Centered, Earth-Fixed) to geodetic coordinates (LLA)
		*/
		GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
		try {
			earth.Reverse(ecef_origin.x(), ecef_origin.y(), ecef_origin.z(),
				map_origin.x(), map_origin.y(), map_origin.z());
		}
		catch (const std::exception& e) {
			ROS_WARN_STREAM("setpoint: Caught exception: " << e.what() << std::endl);
			return;
		}
		is_map_init = true;
	}




	/* -*- rx handler -*- */

	/**
	 * @brief handle POSITION_TARGET_GLOBAL_INT mavlink msg 
	 * handles and publishes position target received from FCU
	 */

	void handle_position_target_global_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::POSITION_TARGET_GLOBAL_INT &position_target)
	{
		/* check if type_mask field ignores position*/
		if (position_target.type_mask & (mavros_msgs::GlobalPositionTarget::IGNORE_LATITUDE | mavros_msgs::GlobalPositionTarget::IGNORE_LONGITUDE)) {
			ROS_WARN_NAMED("setpoint", "lat and/or lon ignored");
			return;
		}

		/* check origin has been set */
		if (!is_map_init) {
			ROS_WARN_NAMED("setpoint", "PositionTargetGlobal failed because no origin");
		}

		/* convert lat/lon target to ECEF */
		Eigen::Vector3d pos_target_ecef {};  //!< local ECEF coordinates on map frame [m]
		GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
		try {
			earth.Forward(position_target.lat_int / 1E7, position_target.lon_int / 1E7, position_target.alt / 1E3,
				pos_target_ecef.x(), pos_target_ecef.y(), pos_target_ecef.z());
		}
		catch (const std::exception& e) {
			ROS_WARN_STREAM("setpoint: Caught exception: " << e.what() << std::endl);
			return;
		}

		/* create position target PoseStamped message */
		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
		pose->header = m_uas->synchronized_header("map", position_target.time_boot_ms);
		pose->pose.orientation.w = 1;   // unit quaternion with no rotation

		/* convert ECEF target to ENU */
		const Eigen::Vector3d local_ecef = pos_target_ecef - ecef_origin;
		tf::pointEigenToMsg(ftf::transform_frame_ecef_enu(local_ecef, map_origin), pose->pose.position);
		pose->pose.position.z = 0;  // force z-axis to zero

		/* publish target */

		if (pose->pose.position.x != arr[0] || pose->pose.position.y != arr[1]) {
			setpointg_pub.publish(pose);
		}

		arr[0] = pose->pose.position.x;
		arr[1] = pose->pose.position.y;
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GuidedTargetPlugin, mavros::plugin::PluginBase)

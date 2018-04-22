/**
 * @brief wind estimation plugin
 * @file wind_estimation.cpp
 * @author Thomas Stastny <thomas.stastny@mavt.ethz.ch>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Thomas Stastny <thomas.stastny@mavt.ethz.ch>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <angles/angles.h>
#include <mavros/mavros_plugin.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Wind estimation plugin.
 */
class WindEstimationPlugin : public plugin::PluginBase {
public:
	WindEstimationPlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		wind_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("wind_estimation", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			make_handler(&WindEstimationPlugin::handle_APM_wind),
			make_handler(&WindEstimationPlugin::handle_PX4_wind),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher wind_pub;

	/**
	 * Handle APM specific wind direction estimation message
	 */
	void handle_APM_wind(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::WIND &wind)
	{
		const double speed = wind.speed;
		const double course = angles::from_degrees(wind.direction);

		auto twist_cov = boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>();
		twist_cov->header.stamp = ros::Time::now();
		// TODO: check math's
		twist_cov->twist.twist.linear.x = speed * std::sin(course);
		twist_cov->twist.twist.linear.y = speed * std::cos(course);
		twist_cov->twist.twist.linear.z = wind.speed_z;

		wind_pub.publish(twist_cov);
	}

	/**
	 * Handle APM specific wind direction estimation message
	 */
	void handle_PX4_wind(const mavlink::mavlink_message_t *msg, mavlink::common::msg::WIND_COV &wind)
	{
		auto twist_cov = boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>();
		twist_cov->header.stamp = ros::Time::now();

		twist_cov->twist.twist.linear.x = wind.wind_x;
		twist_cov->twist.twist.linear.y = wind.wind_y;
		twist_cov->twist.twist.linear.z = wind.wind_z;

		twist_cov->twist.covariance[0] = wind.var_horiz; // NOTE: this is a summed covariance for both x and y horizontal wind components
		twist_cov->twist.covariance[2] = wind.var_vert;

		wind_pub.publish(twist_cov);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WindEstimationPlugin, mavros::plugin::PluginBase)

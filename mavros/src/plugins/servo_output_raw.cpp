/**
 * @brief Servo Output Raw plugin
 * @file servo_output_raw.cpp
 * @author Roman Fedorenko <frontwise@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2020 Roman Fedorenko.
 *
 */
#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/ServoOutputRaw.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Servo Output Raw plugin
 */
class ServoOutputRawPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ServoOutputRawPlugin() : PluginBase(),
		plugin_nh("~")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		servo_output_raw_pub = plugin_nh.advertise<mavros_msgs::ServoOutputRaw>("servo_output_raw", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&ServoOutputRawPlugin::handle_servo_output_raw),
		};
	}

private:
	ros::NodeHandle plugin_nh;

	ros::Publisher servo_output_raw_pub;

	/* -*- rx handlers -*- */

	void handle_servo_output_raw(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SERVO_OUTPUT_RAW &servo_output_raw) {
		auto servo_output_raw_msg = boost::make_shared<mavros_msgs::ServoOutputRaw>();

		servo_output_raw_msg->header.stamp = m_uas->synchronise_stamp(servo_output_raw.time_usec);
		servo_output_raw_msg->port	      = servo_output_raw.port;
		servo_output_raw_msg->servo1_raw  = servo_output_raw.servo1_raw;
		servo_output_raw_msg->servo2_raw  = servo_output_raw.servo2_raw;
		servo_output_raw_msg->servo3_raw  = servo_output_raw.servo3_raw;
		servo_output_raw_msg->servo4_raw  = servo_output_raw.servo4_raw;
		servo_output_raw_msg->servo5_raw  = servo_output_raw.servo5_raw;
		servo_output_raw_msg->servo6_raw  = servo_output_raw.servo6_raw;
		servo_output_raw_msg->servo7_raw  = servo_output_raw.servo7_raw;
		servo_output_raw_msg->servo8_raw  = servo_output_raw.servo8_raw;
		servo_output_raw_msg->servo9_raw  = servo_output_raw.servo9_raw;
		servo_output_raw_msg->servo10_raw = servo_output_raw.servo10_raw;
		servo_output_raw_msg->servo11_raw = servo_output_raw.servo11_raw;
		servo_output_raw_msg->servo12_raw = servo_output_raw.servo12_raw;
		servo_output_raw_msg->servo13_raw = servo_output_raw.servo13_raw;
		servo_output_raw_msg->servo14_raw = servo_output_raw.servo14_raw;
		servo_output_raw_msg->servo15_raw = servo_output_raw.servo15_raw;
		servo_output_raw_msg->servo16_raw = servo_output_raw.servo16_raw;

		servo_output_raw_pub.publish(servo_output_raw_msg);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::ServoOutputRawPlugin, mavros::plugin::PluginBase)

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ServoOutputRaw.h>

namespace mavros {
namespace extra_plugins {
//! @brief Plugin for SERVO_OUTPUT_RAW msgs from MAVLink API
class ServoOutputRawPlugin : public plugin::PluginBase {
public:
	ServoOutputRawPlugin() : PluginBase(),
		servo_output_raw_nh("~servo_output_raw")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		//ros publisher
		servo_output_raw_pub = servo_output_raw_nh.advertise<mavros_msgs::ServoOutputRaw>("servo_output_raw", 10);
	}

	Subscriptions get_subscriptions() {
		return {
			make_handler(&ServoOutputRawPlugin::handle_servo_output_raw)
		};
	}

private:
	ros::NodeHandle servo_output_raw_nh;

	ros::Publisher servo_output_raw_pub;

	/* -*- message handler -*- */

	/**
	 * @brief Handle SERVO_OUTPUT_RAW message.
	 * Message specification: https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW
	 * @param msg	Received Mavlink msg
	 * @param mav_msg	SERVO_OUTPUT_RAW msg
	 */
	void handle_servo_output_raw(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SERVO_OUTPUT_RAW &mav_msg)
	{
		auto ros_msg = boost::make_shared<mavros_msgs::ServoOutputRaw>();
		ros_msg->header.stamp = m_uas->synchronise_stamp(static_cast<uint64_t>(mav_msg.time_usec));
		ros_msg->port = mav_msg.port;
		ros_msg->servo1 = mav_msg.servo1_raw;
		ros_msg->servo2 = mav_msg.servo2_raw;
		ros_msg->servo3 = mav_msg.servo3_raw;
		ros_msg->servo4 = mav_msg.servo4_raw;
		ros_msg->servo5 = mav_msg.servo5_raw;
		ros_msg->servo6 = mav_msg.servo6_raw;
		ros_msg->servo7 = mav_msg.servo7_raw;
		ros_msg->servo8 = mav_msg.servo8_raw;
		ros_msg->servo9 = mav_msg.servo9_raw;
		ros_msg->servo10 = mav_msg.servo10_raw;
		ros_msg->servo11 = mav_msg.servo11_raw;
		ros_msg->servo12 = mav_msg.servo12_raw;
		ros_msg->servo13 = mav_msg.servo13_raw;
		ros_msg->servo14 = mav_msg.servo14_raw;
		ros_msg->servo15 = mav_msg.servo15_raw;
		ros_msg->servo16 = mav_msg.servo16_raw;

		servo_output_raw_pub.publish(ros_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ServoOutputRawPlugin, mavros::plugin::PluginBase)

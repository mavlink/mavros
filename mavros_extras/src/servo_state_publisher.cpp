/**
 * @brief Publish servo states as JointState message
 * @file
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2015 Vladimir Ermakov
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <urdf/model.h>
#include <mavros_msgs/RCOut.h>
#include <sensor_msgs/JointState.h>

class ServoDescription {
public:
	std::string joint_name;
	float joint_lower;
	float joint_upper;

	size_t rc_channel;

	uint16_t rc_min;
	uint16_t rc_max;
	uint16_t rc_trim;
	uint16_t rc_dz;
	bool rc_rev;

	ServoDescription() :
		joint_name{},
		joint_lower(-M_PI/4),
		joint_upper(M_PI/4),
		rc_channel(0),
		rc_min(1000),
		rc_max(2000),
		rc_trim(1500),
		rc_dz(0),
		rc_rev(false)
	{ };

	ServoDescription(std::string joint_name_, double lower_, double upper_,
			int channel_,
			int min_, int max_, int trim_, int dz_,
			bool rev_) :
		joint_name(joint_name_),
		joint_lower(lower_),
		joint_upper(upper_),
		rc_channel(channel_),
		rc_min(min_),
		rc_max(max_),
		rc_trim(trim_),
		rc_dz(dz_),
		rc_rev(rev_)
	{ };

	/**
	 * Normalization code taken from PX4 Firmware
	 * src/modules/sensors/sensors.cpp Sensors::rc_poll() line 1966
	 */
	inline float normalize(uint16_t pwm) {
		// 1) fix bounds
		pwm = std::max(pwm, rc_min);
		pwm = std::min(pwm, rc_max);

		// 2) scale around mid point
		float chan;
		if (pwm > (rc_trim + rc_dz)) {
			chan = (pwm - rc_trim - rc_dz) / (float)(rc_max - rc_trim - rc_dz);
		}
		else if (pwm < (rc_trim - rc_dz)) {
			chan = (pwm - rc_trim + rc_dz) / (float)(rc_trim - rc_min - rc_dz);
		}
		else {
			chan = 0.0;
		}

		if (rc_rev)
			chan *= -1;

		if (!std::isfinite(chan)) {
			ROS_DEBUG("SSP: not finite result in RC%zu channel normalization!", rc_channel);
			chan = 0.0;
		}

		return chan;
	}

	float calculate_position(uint16_t pwm) {
		float channel = normalize(pwm);

		// not sure should i differently map -1..0 and 0..1
		// for now there arduino map() (explicit)
		float position = (channel + 1.0) * (joint_upper - joint_lower) / (1.0 + 1.0) + joint_lower;

		return position;
	}
};

class ServoStatePublisher {
public:
	ServoStatePublisher() :
		nh()
	{
		ros::NodeHandle priv_nh("~");

		XmlRpc::XmlRpcValue param_dict;
		priv_nh.getParam("", param_dict);

		ROS_ASSERT(param_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);

		urdf::Model model;
		model.initParam("robot_description");
		ROS_INFO("SSP: URDF robot: %s", model.getName().c_str());

		for (auto &pair : param_dict) {
			ROS_DEBUG("SSP: Loading joint: %s", pair.first.c_str());

			// inefficient, but easier to program
			ros::NodeHandle pnh(priv_nh, pair.first);

			bool rc_rev;
			int rc_channel, rc_min, rc_max, rc_trim, rc_dz;

			if (!pnh.getParam("rc_channel", rc_channel)) {
				ROS_ERROR("SSP: '%s' should provice rc_channel", pair.first.c_str());
				continue;
			}

			pnh.param("rc_min", rc_min, 1000);
			pnh.param("rc_max", rc_max, 2000);
			if (!pnh.getParam("rc_trim", rc_trim)) {
				rc_trim = rc_min + (rc_max - rc_min) / 2;
			}

			pnh.param("rc_dz", rc_dz, 0);
			pnh.param("rc_rev", rc_rev, false);

			auto joint = model.getJoint(pair.first);
			if (!joint) {
				ROS_ERROR("SSP: URDF: there no joint '%s'", pair.first.c_str());
				continue;
			}
			if (!joint->limits) {
				ROS_ERROR("SSP: URDF: joint '%s' should provide <limit>", pair.first.c_str());
				continue;
			}

			double lower = joint->limits->lower;
			double upper = joint->limits->upper;

			servos.emplace_back(pair.first, lower, upper, rc_channel, rc_min, rc_max, rc_trim, rc_dz, rc_rev);
			ROS_INFO("SSP: joint '%s' (RC%d) loaded", pair.first.c_str(), rc_channel);
		}

		rc_out_sub = nh.subscribe("rc_out", 10, &ServoStatePublisher::rc_out_cb, this);
		joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
	}

	void spin() {
		if (servos.empty()) {
			ROS_WARN("SSP: there nothing to do, exiting");
			return;
		}

		ROS_INFO("SSP: Initialization done. %zu joints served", servos.size());
		ros::spin();
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber rc_out_sub;
	ros::Publisher joint_states_pub;

	std::list<ServoDescription> servos;

	void rc_out_cb(const mavros_msgs::RCOut::ConstPtr &msg) {
		if (msg->channels.empty())
			return;		// nothing to do

		auto states = boost::make_shared<sensor_msgs::JointState>();
		states->header.stamp = msg->header.stamp;

		for (auto &desc : servos) {
			if (!(desc.rc_channel != 0 && desc.rc_channel <= msg->channels.size()))
				continue;	// prevent crash on servos not in that message

			uint16_t pwm = msg->channels[desc.rc_channel - 1];
			if (pwm == 0 || pwm == UINT16_MAX)
				continue;	// exclude unset channels

			states->name.emplace_back(desc.joint_name);
			states->position.emplace_back(desc.calculate_position(pwm));
		}

		joint_states_pub.publish(states);
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "servo_state_publisher");

	ServoStatePublisher state_publisher;
	state_publisher.spin();
	return 0;
}


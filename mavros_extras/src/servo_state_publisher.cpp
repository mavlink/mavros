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
	bool rc_rev;

	ServoDescription() :
		joint_name{},
		joint_lower(-M_PI),
		joint_upper(M_PI),
		rc_channel(0),
		rc_min(1000),
		rc_max(2000),
		rc_trim(1500),
		rc_rev(false)
	{ };

	ServoDescription(std::string joint_name_, double lower_, double upper_,
			int channel_,
			int min_, int max_, int trim_,
			bool rev_) :
		joint_name(joint_name_),
		joint_lower(lower_),
		joint_upper(upper_),
		rc_channel(channel_),
		rc_min(min_),
		rc_max(max_),
		rc_trim(trim_),
		rc_rev(rev_)
	{ };
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

		for (auto &pair : param_dict) {
			ROS_DEBUG("Loading joint: %s", pair.first.c_str());

			// inefficient, but easier to program
			ros::NodeHandle pnh(priv_nh, pair.first);

			int rc_channel, rc_min, rc_max, rc_trim;
			if (!pnh.getParam("rc_channel", rc_channel)) {
				ROS_ERROR("Servo joint: %s should provice rc_channel", pair.first.c_str());
				continue;
			}

			pnh.param("rc_min", rc_min, 1000);
			pnh.param("rc_max", rc_max, 2000);
			if (!pnh.getParam("rc_trim", rc_trim)) {
				rc_trim = rc_min + (rc_max - rc_min) / 2;
			}

			bool rc_rev;
			pnh.param("rc_rev", rc_rev, false);

			// XXX TODO load from URDF lower/upper limits

			servos.emplace_back(pair.first, 0.0, 0.0, rc_channel, rc_min, rc_max, rc_trim, rc_rev);
			ROS_INFO("Servo joint: %s (RC%d) loaded", pair.first.c_str(), rc_channel);
		}
	}

	void spin() {
		ros::spin();
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber rc_out_sub;
	ros::Publisher joint_state_pub;

	std::list<ServoDescription> servos;
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "servo_state_publisher");

	ServoStatePublisher state_publisher;
	state_publisher.spin();
	return 0;
}


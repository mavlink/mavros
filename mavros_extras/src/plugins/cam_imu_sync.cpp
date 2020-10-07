/**
 * @brief Camera IMU synchronisation plugin
 * @file cam_imu_sync.cpp
 * @author Mohammed Kabir < mhkabir98@gmail.com >
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Mohammed Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/CamIMUStamp.h>

namespace mavros {
namespace extra_plugins{
/**
 * @brief Camera IMU synchronisation plugin
 *
 * This plugin publishes a timestamp for when a external camera system was
 * triggered by the FCU. Sequence ID from the message and the image sequence from
 * camera can be corellated to get the exact shutter trigger time.
 */
class CamIMUSyncPlugin : public plugin::PluginBase {
public:
	CamIMUSyncPlugin() : PluginBase(),
		cam_imu_sync_nh("~cam_imu_sync")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		cam_imu_pub = cam_imu_sync_nh.advertise<mavros_msgs::CamIMUStamp>("cam_imu_stamp", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			       make_handler(&CamIMUSyncPlugin::handle_cam_trig)
		};
	}

private:
	ros::NodeHandle cam_imu_sync_nh;

	ros::Publisher cam_imu_pub;

	void handle_cam_trig(const mavlink::mavlink_message_t *msg, mavlink::common::msg::CAMERA_TRIGGER &ctrig)
	{
		auto sync_msg = boost::make_shared<mavros_msgs::CamIMUStamp>();

		sync_msg->frame_stamp = m_uas->synchronise_stamp(ctrig.time_usec);
		sync_msg->frame_seq_id = ctrig.seq;

		cam_imu_pub.publish(sync_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CamIMUSyncPlugin, mavros::plugin::PluginBase)

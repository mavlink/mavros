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
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/CamIMUStamp.h>

namespace mavplugin {
/**
 * @brief Camera IMU synchronisation plugin
 *
 * This plugin publishes a timestamp for when a external camera system was 
 * triggered by the FCU. Sequence ID from the message and the image sequence from
 * camera can be corellated to get the exact shutter trigger time.
 */
class CamIMUSyncPlugin : public MavRosPlugin {
public:
	CamIMUSyncPlugin() :
		cam_imu_sync_nh("~cam_imu_sync"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		cam_imu_pub = cam_imu_sync_nh.advertise<mavros_msgs::CamIMUStamp>("cam_imu_stamp", 10);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_CAMERA_TRIGGER, &CamIMUSyncPlugin::handle_cam_trig)
		};
	}

private:
	ros::NodeHandle cam_imu_sync_nh;
	UAS *uas;

	ros::Publisher cam_imu_pub;

	void handle_cam_trig(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_camera_trigger_t ctrig;
		mavlink_msg_camera_trigger_decode(msg, &ctrig);

		auto sync_msg = boost::make_shared<mavros_msgs::CamIMUStamp>();

		sync_msg->frame_stamp = uas->synchronise_stamp(ctrig.time_usec);
		sync_msg->frame_seq_id = ctrig.seq;

		cam_imu_pub.publish(sync_msg);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::CamIMUSyncPlugin, mavplugin::MavRosPlugin)

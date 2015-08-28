/**
 * @brief ManualControls plugin
 * @file manual_controls.cpp
 * @author Matias Nitsche <mnitsche@dc.uba.ar>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/ManualControls.h>

namespace mavplugin {
/**
 * @brief Manual Controls plugin
 */
class ManualControlsPlugin : public MavRosPlugin {
public:
  ManualControlsPlugin() :
    rc_nh("~manual_controls"),
    uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

    controls_pub = rc_nh.advertise<mavros_msgs::ManualControls>("controls", 10);
    //uas->sig_connection_changed.connect(boost::bind(&RCIOPlugin::connection_cb, this, _1));
	};

	const message_map get_rx_handlers() {
		return {
             MESSAGE_HANDLER(MAVLINK_MSG_ID_MANUAL_CONTROL, &ManualControlsPlugin::handle_manual_control),
		};
	}

private:
	ros::NodeHandle rc_nh;
	UAS *uas;

  ros::Publisher controls_pub;

	/* -*- rx handlers -*- */

  void handle_manual_control(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
    mavlink_manual_control_t manual_control;
    mavlink_msg_manual_control_decode(msg, &manual_control);

    mavros_msgs::ManualControls manual_controls_ros;
    manual_controls_ros.header.stamp = ros::Time::now(); // correct?
    manual_controls_ros.x = manual_control.x;
    manual_controls_ros.y = manual_control.y;
    manual_controls_ros.z = manual_control.z;
    manual_controls_ros.r = manual_control.r;

		controls_pub.publish(manual_controls_ros);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ManualControlsPlugin, mavplugin::MavRosPlugin)


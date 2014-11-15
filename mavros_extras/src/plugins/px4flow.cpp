/**
 * @brief PX4Flow plugin
 * @file px4flow.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 M.H.Kabir.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_extras/OpticalFlow.h>

namespace mavplugin {

/**
 * @brief PX4 Optical Flow plugin
 *
 * This plugin can publish data from PX4Flow camera to ROS
 * and send it to FCU.
 */
class PX4FlowPlugin : public MavRosPlugin {
public:
	PX4FlowPlugin() :
		uas(nullptr)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;

		flow_pub = nh.advertise<mavros_extras::OpticalFlow>("optical_flow", 10);
		flow_rad_pub = nh.advertise<mavros_extras::OpticalFlowRad>("optical_flow_rad", 10);
		twist_pub = nh.advertise<geometry_msgs::Twist>("velocity", 10);
			// distance ranger and temperature too ?

	}

	const std::string get_name() const {
		return "PX4Flow";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_OPTICAL_FLOW, &PX4FlowPlugin::handle_optical_flow)
			MESSAGE_HANDLER(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, &PX4FlowPlugin::handle_optical_flow_rad)
		};
	}

private:
	UAS *uas;

	ros::Publisher flow_pub;
	ros::Subscriber flow_sub;

	void handle_optical_flow(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (flow_pub.getNumSubscribers() == 0)
			return;

		mavlink_optical_flow_t flow;
		mavlink_msg_optical_flow_decode(msg, &flow);

		mavros_extras::OpticalFlowPtr flow_msg =
			boost::make_shared<mavros_extras::OpticalFlow>();

		// Note: for ENU->NED conversion XXX CHECK DIS
		flow_msg->header.stamp = ros::Time::now();
		flow_msg->flow_x = flow.flow_y;
		flow_msg->flow_y = flow.flow_x;
		flow_msg->flow_comp_m_x	 = flow.flow_comp_m_y;
		flow_msg->flow_comp_m_y	 = flow.flow_comp_m_x;
		flow_msg->quality = flow.quality;
		flow_msg->ground_distance = flow.ground_distance;

		flow_pub.publish(flow_msg);

		/* Optional TODO: send ground_distance in sensor_msgs/Range
		 *                with data filled by spec on used sonar.
		 */
	}

	void handle_optical_flow_rad(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (flow_pub.getNumSubscribers() == 0)
			return;

		mavlink_optical_flow_rad_t flow_rad;
		mavlink_msg_optical_flow_rad_decode(msg, &flow_rad);

		mavros_extras::OpticalFlowRadPtr flow_rad_msg =
			boost::make_shared<mavros_extras::OpticalFlowRad>();

		
		flow_rad_msg->header.stamp = ros::Time::now(); // TODO use PX4flow stamp?
		
		// Note: for ENU->NED conversion XXX CHECK DIS
		flow_msg->integration_time_us = flow_rad.integration_time_us;
		flow_msg->integrated_x = flow_rad.integrated_x;
		flow_msg->integrated_y = flow_rad.integrated_y;
		flow_msg->integrated_xgyro = flow_rad.integrated_xgyro;
		flow_msg->integrated_ygyro = flow_rad.integrated_ygyro;
		flow_msg->integrated_zgyro = flow_rad.integrated_zgyro;
		flow_msg->temperature = flow_rad.temperature;
		flow_msg->time_delta_distance_us = flow_rad.time_delta_distance_us
		flow_msg->distance = flow_rad.distance;

		flow_rad_pub.publish(flow_rad_msg);

		/* Optional TODO: send ground_distance in sensor_msgs/Range and temperature too

		 *                with data filled by spec on used sonar.

		 */
	}

};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::PX4FlowPlugin, mavplugin::MavRosPlugin)

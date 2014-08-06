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
	PX4FlowPlugin()
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		bool mode_tx;
		uas = &uas_;

		nh.param("optical_flow_tx", mode_tx, false);
		if (!mode_tx)
			flow_pub = nh.advertise<mavros_extras::OpticalFlow>("optical_flow", 10);
		else
			flow_sub = nh.subscribe("optical_flow", 10, &PX4FlowPlugin::send_flow_cb, this);
	}

	const std::string get_name() const {
		return "PX4Flow";
	}

	const std::vector<uint8_t> get_supported_messages() const {
		return {
			MAVLINK_MSG_ID_OPTICAL_FLOW
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		handle_flow(msg);
	}

private:
	UAS *uas;

	ros::Publisher flow_pub;
	ros::Subscriber flow_sub;

	void handle_flow(const mavlink_message_t *msg) {
		if (flow_pub.getNumSubscribers() == 0)
			return;

		mavlink_optical_flow_t flow;
		mavlink_msg_optical_flow_decode(msg, &flow);

		mavros_extras::OpticalFlowPtr flow_msg =
			boost::make_shared<mavros_extras::OpticalFlow>();

		// Note: for ENU->NED conversion i swap x & y.
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

	void optical_flow(uint64_t time_usec, uint8_t sensor_id,
			uint16_t flow_x, uint16_t flow_y,
			float flow_comp_m_x, float flow_comp_m_y,
			uint8_t quality,
			float ground_distance) {
		mavlink_message_t msg;
		mavlink_msg_optical_flow_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_usec, sensor_id,
				flow_x, flow_y,
				flow_comp_m_x, flow_comp_m_y,
				quality, ground_distance);
		uas->mav_link->send_message(&msg);
	}

	/* -*- ROS callbacks -*- */

	void send_flow_cb(const mavros_extras::OpticalFlow::ConstPtr flow_msg) {
		optical_flow(flow_msg->header.stamp.toNSec() / 1000,
				0, /* maybe we need parameter? */
				flow_msg->flow_y,
				flow_msg->flow_x,
				flow_msg->flow_comp_m_y,
				flow_msg->flow_comp_m_x,
				flow_msg->quality,
				flow_msg->ground_distance);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::PX4FlowPlugin, mavplugin::MavRosPlugin)

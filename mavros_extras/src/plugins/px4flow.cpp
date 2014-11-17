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
#include <mavros_extras/OpticalFlowRad.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>

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

		flow_nh = ros::NodeHandle(nh, "px4flow");
		
		flow_nh.param<std::string>("frame_id", frame_id, "px4flow");

		//Default rangefinder is Maxbotix HRLV-EZ4
		flow_nh.param("ranger_type", ranger_type, 0);	// See sensor_msgs/Range docs 
		flow_nh.param("ranger_fov", ranger_fov, 0.0); 	// TODO
		flow_nh.param("ranger_min_range", ranger_min_range, 0.3);	
		flow_nh.param("ranger_max_range", ranger_max_range, 5.0);

		flow_pub = flow_nh.advertise<mavros_extras::OpticalFlow>("raw/optical_flow", 10);
		flow_rad_pub = flow_nh.advertise<mavros_extras::OpticalFlowRad>("raw/optical_flow_rad", 10);
		twist_pub = flow_nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity", 10);
		range_pub = flow_nh.advertise<sensor_msgs::Range>("ground_distance", 10);
		temp_pub = flow_nh.advertise<sensor_msgs::Temperature>("temperature", 10);

	}

	const std::string get_name() const {
		return "PX4Flow";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, &PX4FlowPlugin::handle_optical_flow_rad),
			MESSAGE_HANDLER(MAVLINK_MSG_ID_OPTICAL_FLOW, &PX4FlowPlugin::handle_optical_flow)
		};
	}

private:
	UAS *uas;

	std::string frame_id;
	
	int ranger_type;
	double ranger_fov;
	double ranger_min_range;
	double ranger_max_range;

	ros::Publisher flow_pub;
	ros::Publisher flow_rad_pub;
	ros::Publisher twist_pub;
	ros::Publisher range_pub;
	ros::Publisher temp_pub;

	void handle_optical_flow(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	
		mavlink_optical_flow_t flow;
		mavlink_msg_optical_flow_decode(msg, &flow);

		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = frame_id;

		mavros_extras::OpticalFlowPtr flow_msg =
			boost::make_shared<mavros_extras::OpticalFlow>();
	
		flow_msg->header = header;

		flow_msg->flow_x = flow.flow_x;
		flow_msg->flow_y = -flow.flow_y; //NED -> ENU
		flow_msg->flow_comp_m_x	 = flow.flow_comp_m_x;
		flow_msg->flow_comp_m_y	 = -flow.flow_comp_m_y; //NED -> ENU
		flow_msg->quality = flow.quality;
		flow_msg->ground_distance = flow.ground_distance;

		flow_pub.publish(flow_msg);
	}
	
	void handle_optical_flow_rad(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

		mavlink_optical_flow_rad_t flow_rad;
		mavlink_msg_optical_flow_rad_decode(msg, &flow_rad);
	
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = frame_id;

		// Raw message with axes mapped to ROS conventions and temp in degrees celsius
		mavros_extras::OpticalFlowRadPtr flow_rad_msg =
			boost::make_shared<mavros_extras::OpticalFlowRad>();
		
		flow_rad_msg->header = header; 
		
		flow_rad_msg->integration_time_us = flow_rad.integration_time_us;
		flow_rad_msg->integrated_x = flow_rad.integrated_x;
		flow_rad_msg->integrated_y = -flow_rad.integrated_y; // NED -> ENU
		flow_rad_msg->integrated_xgyro = flow_rad.integrated_xgyro;
		flow_rad_msg->integrated_ygyro = -flow_rad.integrated_ygyro; // NED -> ENU
		flow_rad_msg->integrated_zgyro = -flow_rad.integrated_zgyro; // NED -> ENU
		flow_rad_msg->temperature = flow_rad.temperature/100.0f; // in degrees celsius
		flow_rad_msg->time_delta_distance_us = flow_rad.time_delta_distance_us;
		flow_rad_msg->distance = flow_rad.distance; 

		flow_rad_pub.publish(flow_rad_msg);
				
		if(flow_rad.quality > 0) // don't publish invalid data
		{
		//Twist
		geometry_msgs::TwistWithCovarianceStampedPtr twist_msg =
		boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>();

		twist_msg->header = header; 
		
		twist_msg->twist.twist.linear.x = ((flow_rad.integrated_x/flow_rad.integration_time_us)/flow_rad.distance)*1000000 ;
		twist_msg->twist.twist.linear.y = -((flow_rad.integrated_y/flow_rad.integration_time_us)/flow_rad.distance)*1000000;// NED -> ENU
		twist_msg->twist.twist.linear.z = 0; //from sonar?
		twist_msg->twist.twist.angular.x = (flow_rad.integrated_xgyro/flow_rad.integration_time_us)*1000000;
		twist_msg->twist.twist.angular.y = -(flow_rad.integrated_ygyro/flow_rad.integration_time_us)*1000000;// NED -> ENU
		twist_msg->twist.twist.angular.z = -(flow_rad.integrated_zgyro/flow_rad.integration_time_us)*1000000;// NED -> ENU

		//TODO : Covariances from quality?

		twist_pub.publish(twist_msg);

		}

		// Temperature
		sensor_msgs::TemperaturePtr temp_msg = 
			boost::make_shared<sensor_msgs::Temperature>();
		
		temp_msg->header = header;	

		temp_msg->temperature = flow_rad.temperature/100.0f; 
		
		temp_pub.publish(temp_msg);

		// Rangefinder
		sensor_msgs::RangePtr range_msg = 
			boost::make_shared<sensor_msgs::Range>();

		range_msg->header = header;
		
		range_msg->radiation_type = ranger_type;
		range_msg->field_of_view = ranger_fov;
		range_msg->min_range = ranger_min_range;
		range_msg->max_range = ranger_max_range;
		range_msg->range = flow_rad.distance;

		range_pub.publish(range_msg);
	}

};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::PX4FlowPlugin, mavplugin::MavRosPlugin)

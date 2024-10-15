/**
 * @brief Rally Point plugin
 * @file rally_point.cpp
 * @author Karthik Desai <karthik.desai@iav.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2020 Karthik Desai
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>

#include <mavros_msgs/RallyPoint.h>
#include <mavros_msgs/RallyPointGet.h>
#include <mavros_msgs/RallyPointSet.h>

namespace mavros {
namespace extra_plugins {

class RallyPointPlugin : public plugin::PluginBase {
public:
    RallyPointPlugin():
        nh("~rally_point") {}
    void initialize(UAS& uas) override
	{
		PluginBase::initialize(uas);

		rally_point_pub = nh.advertise<mavros_msgs::RallyPoint>("point", 5);

		rally_point_request_srv = nh.advertiseService("get",
					&RallyPointPlugin::rally_point_request_cb, this);
		rally_point_set_srv = nh.advertiseService("set",
					&RallyPointPlugin::rally_point_set_cb, this);
	}

    Subscriptions get_subscriptions() override
	{
		return {
			       make_handler(&RallyPointPlugin::handle_rally_point),
		};
	}
private:
	ros::NodeHandle nh;
	ros::Publisher rally_point_pub;
	ros::ServiceServer rally_point_request_srv, rally_point_set_srv;

    void handle_rally_point(const mavlink::mavlink_message_t*, mavlink::ardupilotmega::msg::RALLY_POINT& rp)
	{
		auto msg = boost::make_shared<mavros_msgs::RallyPoint>();
		msg->header.stamp = ros::Time::now();
        msg->idx = rp.idx;
        msg->count = rp.count;
        msg->latitude = rp.lat / 1e7;
        msg->longitude = rp.lng / 1e7;
        msg->altitude = rp.alt;
        msg->break_altitude = rp.break_alt;
        msg->land_direction = rp.land_dir;
        msg->flags = rp.flags;

		rally_point_pub.publish(msg);
	}

    bool rally_point_request_cb(mavros_msgs::RallyPointGet::Request &req,
    mavros_msgs::RallyPointGet::Response &resp)
	{
        mavlink::ardupilotmega::msg::RALLY_FETCH_POINT msg;
		m_uas->msg_set_target(msg);
        msg.idx = req.idx;
        resp.success = true;
		try {
			UAS_FCU(m_uas)->send_message(msg);
		} catch (std::length_error&) {
			resp.success = false;
		}
		return true;
	}

    bool rally_point_set_cb(mavros_msgs::RallyPointSet::Request &req,
    mavros_msgs::RallyPointSet::Response &resp)
	{
        mavlink::ardupilotmega::msg::RALLY_POINT msg;
		m_uas->msg_set_target(msg);
        msg.idx = req.rally.idx;
        msg.count = req.rally.count;
        msg.lat = req.rally.latitude * 1e7;
        msg.lng = req.rally.longitude * 1e7;
        msg.alt = req.rally.altitude;
        msg.break_alt = req.rally.break_altitude;
        msg.land_dir = req.rally.land_direction;
        msg.flags = req.rally.flags;
        resp.success = true;
		try {
			UAS_FCU(m_uas)->send_message(msg);
		} catch (std::length_error&) {
			resp.success = false;
		}
		return true;
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::RallyPointPlugin, mavros::plugin::PluginBase)
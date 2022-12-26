/**
 * @brief NavControllerOutput plugin
 * @file nav_controller_output.cpp
 * @author Karthik Desai <karthik.desai@iav.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2019 Karthik Desai
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include "mavros_msgs/NavControllerOutput.h"

namespace mavros
{
namespace std_plugins
{
/**
 * @brief nav controller output plugin.
 *
 * Publishes nav_controller_output message https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT
 */
class NavControllerOutputPlugin : public plugin::PluginBase
{
public:
    NavControllerOutputPlugin() : PluginBase(),
                                  nh("~")
    {
    }

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        nco_pub = nh.advertise<mavros_msgs::NavControllerOutput>("nav_controller_output", 10);
    }

    Subscriptions get_subscriptions()
    {
        return {
            make_handler(&NavControllerOutputPlugin::handle_nav_controller_output),
        };
    }

private:
    ros::NodeHandle nh;

    ros::Publisher nco_pub;

    void handle_nav_controller_output(const mavlink::mavlink_message_t *msg, mavlink::common::msg::NAV_CONTROLLER_OUTPUT &nav_controller_output)
    {
        auto nco_msg = boost::make_shared<mavros_msgs::NavControllerOutput>();
        nco_msg->header.stamp = ros::Time::now();
        nco_msg->nav_roll = nav_controller_output.nav_roll;
        nco_msg->nav_pitch = nav_controller_output.nav_pitch;
        nco_msg->nav_bearing = nav_controller_output.nav_bearing;
        nco_msg->target_bearing = nav_controller_output.target_bearing;
        nco_msg->wp_dist = nav_controller_output.wp_dist;
        nco_msg->alt_error = nav_controller_output.alt_error;
        nco_msg->aspd_error = nav_controller_output.aspd_error;
        nco_msg->xtrack_error = nav_controller_output.xtrack_error;

        nco_pub.publish(nco_msg);
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::NavControllerOutputPlugin, mavros::plugin::PluginBase)

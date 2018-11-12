/**
 * @brief GCSBridge plugin
 * @file gcs_bridge_plugin.cpp
 * @author Amy Wagoner <arwagoner@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Amy Wagoner.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>


namespace mavros {
namespace extra_plugins{
/**
 * @brief GCS Bridge plugin
 *
 *
 */
class GCSBridgePlugin : public plugin::PluginBase {
public:
    GCSBridgePlugin() : PluginBase(),
    mavlink_nh("mavlink")
	{ }

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
                
        mavlink_sub = mavlink_nh.subscribe("from", 10, &GCSBridgePlugin::mavlink_sub_cb,this);
        mavlink_pub = mavlink_nh.advertise<mavros_msgs::Mavlink>("to", 10);
    }

    Subscriptions get_subscriptions()
    {
        return { /* Rx disabled */ };
    }

private:
    ros::NodeHandle mavlink_nh;

    ros::Subscriber mavlink_sub;
    ros::Publisher mavlink_pub;
        
    void mavlink_pub_cb(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing)
    {
        auto rmsg = boost::make_shared<mavros_msgs::Mavlink>();

        rmsg->header.stamp = ros::Time::now();
        mavros_msgs::mavlink::convert(*mmsg, *rmsg, mavros::utils::enum_value(framing));
        mavlink_pub.publish(rmsg);
    }

    void mavlink_sub_cb(const mavros_msgs::Mavlink::ConstPtr &rmsg)
    {
        mavlink::mavlink_message_t mmsg;
        if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
        {
            UAS_GCS(m_uas)->send_message_ignore_drop(&mmsg);
        }
        else
            ROS_ERROR("Packet drop: convert error.");
    }
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GCSBridgePlugin, mavros::plugin::PluginBase)

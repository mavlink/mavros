/**
 * @brief Altitude plugin
 * @file altitude.cpp
 * @author Andreas Antener <andreas@uaventure.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Andreas Antener <andreas@uaventure.com>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/Altitude.h>

namespace mavplugin {
/**
 * @brief Altitude plugin.
 */
class AltitudePlugin : public MavRosPlugin {
public:
    AltitudePlugin() :
        nh("~"),
        uas(nullptr)
    { }

    /**
     * Plugin initializer. Constructor should not do this.
     */
    void initialize(UAS &uas_)
    {
        uas = &uas_;
        nh.param<std::string>("frame_id", frame_id, "map");
        altitude_pub = nh.advertise<mavros_msgs::Altitude>("altitude", 10);
    }

    const message_map get_rx_handlers() {
        return {
                   MESSAGE_HANDLER(MAVLINK_MSG_ID_ALTITUDE, &AltitudePlugin::handle_altitude),
        };
    }

private:
    ros::NodeHandle nh;
    UAS *uas;
    std::string frame_id;

    ros::Publisher altitude_pub;

    void handle_altitude(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_altitude_t altitude;
        mavlink_msg_altitude_decode(msg, &altitude);

        auto ros_msg = boost::make_shared<mavros_msgs::Altitude>();
        ros_msg->header = uas->synchronized_header(frame_id, altitude.time_usec);
        
        ros_msg->monotonic = altitude.altitude_monotonic;
        ros_msg->amsl = altitude.altitude_amsl;
        ros_msg->local = altitude.altitude_local;
        ros_msg->relative = altitude.altitude_relative;
        ros_msg->terrain = altitude.altitude_terrain;
        ros_msg->bottom_clearance = altitude.bottom_clearance;

        altitude_pub.publish(ros_msg);
    }

};
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::AltitudePlugin, mavplugin::MavRosPlugin)


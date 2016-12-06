/**
 * @brief HilGPS plugin
 * @file hil_gps.cpp
 * @author Mohamed Abdelkader <mohamedashraf123@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 Mohamed Abdelkader.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/hil_sensor_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/HilGPS.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief HIL GPS plugin
 *
 * Send HIL GPS to FCU controller.
 */
class HilGPSPlugin : public plugin::PluginBase,
    private plugin::SetHilGPSMixin<HilGPSPlugin> {
public:
	HilGPSPlugin() : PluginBase(),
		gps_nh("~hil_gps")

	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);


		hilGPS_sub = gps_nh.subscribe("hilgps", 10, &HilGPSPlugin::gps_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetHilGPSMixin;
        
	ros::NodeHandle gps_nh;

	ros::Subscriber hilGPS_sub;


	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send hil_gps to FCU.
	 *
	 * @warning
	 */
	void send_hil_gps(const ros::Time &stamp,
                         uint8_t fix_type,
			int32_t lat, int32_t lon, int32_t alt,
			uint16_t eph, uint16_t epv, uint16_t vel, 
			int16_t vn, int16_t ve, int16_t vd,
			uint16_t cog,
			uint8_t satellites_visible) {

		set_hil_gps(stamp.toNSec() / 1000,
			fix_type,
			lat, lon, alt,
			eph, epv, vel, 
			vn, ve, vd,
			cog,
			satellites_visible);
	}

	/* -*- callbacks -*- */
        

	void gps_cb(const mavros_msgs::HilGPS::ConstPtr &req) {
            
            send_hil_gps(req->header.stamp,
                            req->fix_type,
                            req->lat, req->lon, req->alt,
                            req->eph, req->epv, req->vel,
                            req->vn, req->ve, req->vd,
                            req->cog,
                            req->satellites_visible);
        }
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HilGPSPlugin, mavros::plugin::PluginBase)

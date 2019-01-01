/**
 * @brief Vehicule Info plugin
 * @file vehicle_info.cpp
 * @author Gregoire Linard <gregoire.linard@azurdrones.com>
 *
 * @addtogroup plugin
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include "mavros_msgs/VehicleInfo.h"
#include "mavros_msgs/VehicleInfoGet.h"
#include "mavros_msgs/VehicleInfoGetAll.h"
#include "mavros_msgs/VehicleInfoGetById.h"
#include <std_srvs/Trigger.h>

namespace mavros {
namespace extra_plugins {

using mavlink::common::MAV_AUTOPILOT;
using mavlink::common::MAV_PROTOCOL_CAPABILITY;
using mavlink::common::MAV_STATE;
using mavlink::common::MAV_TYPE;
using utils::enum_value;

/**
 * @brief  Vehicle Info monitor plugin.
 *
 * This plugin allows requesting standard vehicle information.
 * It stores all vehicles, you can request all vehicles or a only specific id.
 *
 */
class VehicleInfoPlugin : public plugin::PluginBase {
  public:

	VehicleInfoPlugin() : PluginBase(),
						  vehicle_info_nh("~")
	{	}

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		vehicle_info_get_srv = vehicle_info_nh.advertiseService("vehicle_info_get", &VehicleInfoPlugin::vehicle_info_get_cb, this);
		vehicle_info_get_by_id_srv = vehicle_info_nh.advertiseService("vehicle_info_get_by_id", &VehicleInfoPlugin::vehicle_info_get_by_id_cb, this);
		vehicle_info_get_all_srv = vehicle_info_nh.advertiseService("vehicle_info_get_all", &VehicleInfoPlugin::vehicle_info_get_all_cb, this);
		vehicle_info_reset_srv = vehicle_info_nh.advertiseService("vehicle_info_reset", &VehicleInfoPlugin::vehicle_info_reset_cb, this);

		my_vehicle.sysid = m_uas->get_tgt_system();
		my_vehicle.compid = m_uas->get_tgt_component();
		vehicles.push_back(my_vehicle);
	}

	Subscriptions get_subscriptions()
	{
		return {
			make_handler(&VehicleInfoPlugin::handle_heartbeat),
			make_handler(&VehicleInfoPlugin::handle_autopilot_version)};
	}

  private:
	ros::NodeHandle vehicle_info_nh;

	ros::ServiceServer vehicle_info_get_srv;
	ros::ServiceServer vehicle_info_get_by_id_srv;
	ros::ServiceServer vehicle_info_get_all_srv;
	ros::ServiceServer vehicle_info_reset_srv;

	struct VehicleInfo {
		uint8_t autopilot = enum_value(MAV_AUTOPILOT::GENERIC); 
		uint8_t type = enum_value(MAV_TYPE::GENERIC);	  
		uint8_t sysid = 0;
		uint8_t compid = 0;
		std::string mode = "";
		uint32_t mode_id = 0;
		uint8_t  system_status = enum_value(MAV_STATE::UNINIT); 
		uint64_t capabilities = 0;
		uint32_t flight_sw_version = 0;
		uint32_t middleware_sw_version = 0;
		uint32_t os_sw_version = 0;
		uint32_t board_version = 0;
		uint16_t vendor_id = 0;
		uint16_t product_id = 0;
		uint64_t uid = 0;
	};
	
	VehicleInfo my_vehicle;
	std::vector<VehicleInfo> vehicles;

	/* -*- message handlers -*- */
	void handle_heartbeat(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HEARTBEAT &hb)
	{
		using mavlink::common::MAV_MODE_FLAG;

		//Fill vehicle if exists
		for (auto &it : vehicles) {
			if(it.sysid == msg->sysid && it.compid == msg->compid){
				
				//TODO handle iconflict where different vehicle autopilot/type but with same sysid/compid
				
				it.autopilot = hb.autopilot;
				it.type = hb.type;
				//update vehicle info
				it.system_status = hb.system_status;
				it.mode = m_uas->str_mode_v10(hb.base_mode, hb.custom_mode);				
				if (!(hb.base_mode & enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED))) {
					it.mode_id = hb.base_mode;
				} else {
					it.mode_id = hb.custom_mode;
				}				
				
				return;
			}
		}		
		
		//or create new vehicle
		VehicleInfo new_vehicle;
		new_vehicle.sysid = msg->sysid;
		new_vehicle.compid = msg->compid;
		new_vehicle.autopilot = hb.autopilot;
		new_vehicle.type = hb.type;
		new_vehicle.mode = m_uas->str_mode_v10(hb.base_mode, hb.custom_mode);
		
		if (!(hb.base_mode & enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED))) {
			new_vehicle.mode_id = hb.base_mode;
		} else {
			new_vehicle.mode_id = hb.custom_mode;
		}

		new_vehicle.system_status = hb.system_status;
		vehicles.push_back(new_vehicle);	
		

	}

	void handle_autopilot_version(const mavlink::mavlink_message_t *msg, mavlink::common::msg::AUTOPILOT_VERSION &apv)
	{
		//Fill vehicle if exists
		for (auto &it : vehicles) {
			if(it.sysid == msg->sysid && it.compid == msg->compid){
				it.capabilities = apv.capabilities;
				it.flight_sw_version = apv.flight_sw_version;
				// custom_version_to_hex_string(apv.flight_custom_version).c_str());
				it.middleware_sw_version = apv.middleware_sw_version;
				//custom_version_to_hex_string(apv.middleware_custom_version).c_str());
				it.os_sw_version = apv.os_sw_version;
				//custom_version_to_hex_string(apv.os_custom_version).c_str());
				it.board_version = apv.board_version;
				it.vendor_id = apv.vendor_id;
				it.product_id = apv.product_id;
				it.uid = apv.uid;
			}
		}

		//or create new vehicle
		VehicleInfo new_vehicle;
		new_vehicle.sysid = msg->sysid;
		new_vehicle.compid = msg->compid;
		new_vehicle.capabilities = apv.capabilities;
		new_vehicle.flight_sw_version = apv.flight_sw_version;
		new_vehicle.middleware_sw_version = apv.middleware_sw_version;
		new_vehicle.os_sw_version = apv.os_sw_version;
		new_vehicle.board_version = apv.board_version;
		new_vehicle.vendor_id = apv.vendor_id;
		new_vehicle.product_id = apv.product_id;
		new_vehicle.uid = apv.uid;
		
		vehicles.push_back(new_vehicle);	

	}

	/* -*-ROS callbacks -*- */

	bool vehicle_info_get_cb(mavros_msgs::VehicleInfoGet::Request &req,
						 mavros_msgs::VehicleInfoGet::Response &res)
	{

		for (auto &it : vehicles) {
			if(it.sysid == my_vehicle.sysid && it.compid == my_vehicle.compid){		
				res.vehicle = it;
				res.success = true;
				return res.success;
			}
		}

		res.success = false;
		return res.success;
	}

	bool vehicle_info_get_by_id_cb(mavros_msgs::VehicleInfoGetById::Request &req,
						 mavros_msgs::VehicleInfoGetById::Response &res)
	{

		for (auto &it : vehicles) {
			if(it.sysid == req.sysid && it.compid == req.compid){

				res.vehicle = it;
				/*
				res.vehicle.type = it.type;
				res.vehicle.autopilot = it.autopilot;
				res.vehicle.mode = it.mode;
				res.vehicle.mode_id = it.mode_id;
				res.vehicle.system_status = it.system_status;
				res.vehicle.sysid = it.sysid;
				res.vehicle.compid = it.compid; 
				res.vehicle.capabilities = it.capabilities;
				res.vehicle.flight_sw_version = it.flight_sw_version;
				res.vehicle.middleware_sw_version = it.middleware_sw_version;
				res.vehicle.os_sw_version = it.os_sw_version;
				res.vehicle.board_version = it.board_version;
				res.vehicle.vendor_id = it.vendor_id;
				res.vehicle.product_id = it.product_id;
				res.vehicle.uid = it.uid;
				*/

				res.success = true;
				return res.success;
			}
		}

		res.success = false;
		return res.success;
	}

	bool vehicle_info_get_all_cb(mavros_msgs::VehicleInfoGetAll::Request &req,
						 mavros_msgs::VehicleInfoGetAll::Response &res)
	{
		for (auto &it : vehicles) {
				res.vehicles.push_back(it);
		}

		res.success = true;
		return res.success;
	}

	bool vehicle_info_reset_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
	{
		vehicles.clear();
		res.success = true;
		return true;
	}


};
} // namespace extra_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VehicleInfoPlugin, mavros::plugin::PluginBase)

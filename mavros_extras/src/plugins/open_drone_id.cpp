/*
 * Copyright 2024 Gus Meyer.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Open Drone ID plugin to satisfy remote ID requirements
 * @file open_drone_id.cpp
 * @author Gus Meyer <gus@robotics88.com>
 *
 * @addtogroup plugin
 * @{
 */


#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/basic_id.hpp"
#include "mavros_msgs/msg/operator_id.hpp"
#include "mavros_msgs/msg/self_id.hpp"
#include "mavros_msgs/msg/system.hpp"
#include "mavros_msgs/msg/system_update.hpp"

namespace mavros {
namespace extra_plugins {
using namespace std::placeholders;      // NOLINT

/**
 * @brief Open Drone ID plugin
 *
 * Sends Open Drone ID data to the FCU
 */
class OpenDroneIDPlugin : public plugin::Plugin 
{
public:
	explicit OpenDroneIDPlugin(plugin::UASPtr uas_) 
	: Plugin(uas_, "open_drone_id")
	{ 
		basic_id_sub = node->create_subscription<mavros_msgs::msg::BasicID>(
			"~/basic_id", 1, std::bind(
			&OpenDroneIDPlugin::basic_id_cb, this,
			_1));

		operator_id_sub = node->create_subscription<mavros_msgs::msg::OperatorID>(
			"~/operator_id", 1, std::bind(
			&OpenDroneIDPlugin::operator_id_cb, this,
			_1));

		self_id_sub = node->create_subscription<mavros_msgs::msg::SelfID>(
			"~/self_id", 1, std::bind(
			&OpenDroneIDPlugin::self_id_cb, this,
			_1));

		system_sub = node->create_subscription<mavros_msgs::msg::System>(
			"~/system", 1, std::bind(
			&OpenDroneIDPlugin::system_cb, this,
			_1));

		system_update_sub = node->create_subscription<mavros_msgs::msg::SystemUpdate>(
			"~/system_update", 1, std::bind(
			&OpenDroneIDPlugin::system_update_cb, this,
			_1));
	}

	Subscriptions get_subscriptions()
	{
		return { };
	}

private:
	rclcpp::Subscription<mavros_msgs::msg::BasicID>::SharedPtr basic_id_sub;
	rclcpp::Subscription<mavros_msgs::msg::OperatorID>::SharedPtr operator_id_sub;
	rclcpp::Subscription<mavros_msgs::msg::SelfID>::SharedPtr self_id_sub;
	rclcpp::Subscription<mavros_msgs::msg::System>::SharedPtr system_sub;
	rclcpp::Subscription<mavros_msgs::msg::SystemUpdate>::SharedPtr system_update_sub;


	void basic_id_cb(const mavros_msgs::msg::BasicID::SharedPtr msg)
	{
		mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID basic_id{};

		basic_id.id_type = msg->id_type;
		basic_id.ua_type = msg->ua_type;

		size_t length = std::min(basic_id.uas_id.size(), msg->uas_id.size());
		std::memcpy(basic_id.uas_id.data(), msg->uas_id.data(), length);

		uas->send_message(basic_id);
	}

	void operator_id_cb(const mavros_msgs::msg::OperatorID::SharedPtr msg)
	{
		mavlink::common::msg::OPEN_DRONE_ID_OPERATOR_ID operator_id{};

		operator_id.operator_id_type = msg->operator_id_type;

		size_t length = std::min(operator_id.operator_id.size(), msg->operator_id.size());
		std::memcpy(operator_id.operator_id.data(), msg->operator_id.data(), length);

		uas->send_message(operator_id);
	}

	void self_id_cb(const mavros_msgs::msg::SelfID::SharedPtr msg) 
	{
		mavlink::common::msg::OPEN_DRONE_ID_SELF_ID self_id{};
		self_id.description_type = msg->description_type;

		size_t length = std::min(self_id.description.size(), msg->description.size());
		std::memcpy(self_id.description.data(), msg->description.data(), length);

		uas->send_message(self_id);
	}

	void system_cb(const mavros_msgs::msg::System::SharedPtr msg) 
	{
		mavlink::common::msg::OPEN_DRONE_ID_SYSTEM system{};

		system.operator_location_type = msg->operator_location_type;
		system.classification_type = msg->classification_type;
		system.operator_latitude = msg->operator_latitude;
		system.operator_longitude = msg->operator_longitude;
		system.area_count = msg->area_count;
		system.area_radius = msg->area_radius;
		system.area_ceiling = msg->area_ceiling;
		system.area_floor = msg->area_floor;
		system.category_eu = msg->category_eu;
		system.class_eu = msg->class_eu;
		system.operator_altitude_geo = msg->operator_altitude_geo;
		system.timestamp = msg->timestamp;

		uas->send_message(system);
	}

	void system_update_cb(const mavros_msgs::msg::SystemUpdate::SharedPtr msg) 
	{
		mavlink::common::msg::OPEN_DRONE_ID_SYSTEM_UPDATE system_update{};

		system_update.operator_latitude = msg->operator_latitude;
		system_update.operator_longitude = msg->operator_longitude;
		system_update.operator_altitude_geo = msg->operator_altitude_geo;
		system_update.timestamp = msg->timestamp;

		uas->send_message(system_update);
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::OpenDroneIDPlugin)

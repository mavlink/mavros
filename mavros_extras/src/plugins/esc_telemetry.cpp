/**
 * @brief APM ESC Telemetry plugin
 * @file esc_telemetry.cpp
 * @author Braedon O'Meara <braedon@rizse.io>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2021 Braedon O'Meara <braedon@rizse.io>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/ESCTelemetry.h>

namespace mavros
{
namespace extra_plugins
{
/**
 * @brief ESC telemetry plugin
 *
 * APM specific plugin.
 */
class ESCTelemetryPlugin : public plugin::PluginBase
{
public:
	ESCTelemetryPlugin() : PluginBase(),
		nh("~")
	{}

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		esc_telemetry_pub = nh.advertise<mavros_msgs::ESCTelemetry>("esc_telemetry", 10);

		enable_connection_cb();
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&ESCTelemetryPlugin::handle_esc_telemetry_1_to_4),
			make_handler(&ESCTelemetryPlugin::handle_esc_telemetry_5_to_8),
			make_handler(&ESCTelemetryPlugin::handle_esc_telemetry_9_to_12),
		};
	}

private:
	using lock_guard = std::lock_guard<std::mutex>;
	std::mutex mutex;

	ros::NodeHandle nh;

	ros::Publisher esc_telemetry_pub;
	mavros_msgs::ESCTelemetry _esc_telemetry;

	template <typename msgT>
	void handle_esc_telemetry(const mavlink::mavlink_message_t *msg, msgT &et, size_t offset = 0)
	{
		lock_guard lock(mutex);

		size_t requred_size = offset + et.temperature.size();
		if (_esc_telemetry.esc_telemetry.size() < requred_size) {
			_esc_telemetry.esc_telemetry.resize(requred_size);
		}

		auto stamp = ros::Time::now();

		_esc_telemetry.header.stamp = stamp;
		for (size_t i = 0; i < et.temperature.size(); i++) {
			auto &p = _esc_telemetry.esc_telemetry.at(offset + i);

			p.header.stamp = stamp;
			p.temperature = et.temperature[i];
			p.voltage = et.voltage[i] / 100.0f;			// centiV -> V
			p.current = et.current[i] / 100.0f;			// centiA -> A
			p.totalcurrent = et.totalcurrent[i] / 1000.0f;		// mAh -> Ah
			p.rpm = et.rpm[i];
			p.count = et.count[i];
		}

		esc_telemetry_pub.publish(_esc_telemetry);
	}

	void handle_esc_telemetry_1_to_4(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::ESC_TELEMETRY_1_TO_4 &esc_telemetry)
	{
		handle_esc_telemetry(msg, esc_telemetry, 0);
	}

	void handle_esc_telemetry_5_to_8(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::ESC_TELEMETRY_5_TO_8 &esc_telemetry)
	{
		handle_esc_telemetry(msg, esc_telemetry, 4);
	}

	void handle_esc_telemetry_9_to_12(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::ESC_TELEMETRY_9_TO_12 &esc_telemetry)
	{
		handle_esc_telemetry(msg, esc_telemetry, 8);
	}

	void connection_cb(bool connected) override
	{
		lock_guard lock(mutex);

		_esc_telemetry.esc_telemetry.clear();
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ESCTelemetryPlugin, mavros::plugin::PluginBase)

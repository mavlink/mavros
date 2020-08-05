/**
 * @brief ESC status plugin
 * @file esc_status.cpp
 * @author Ricardo Marques <marques.ricardo17@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2020 Ricardo Marques <marques.ricardo17@gmail.com>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/ESCInfo.h>
#include <mavros_msgs/ESCStatus.h>

namespace mavros
{
namespace extra_plugins
{
/**
 * @brief ESC status plugin
 */
class ESCStatusPlugin : public plugin::PluginBase
{
public:
	ESCStatusPlugin() : PluginBase(),
		_max_esc_count(0),
		_max_esc_info_index(0),
		_max_esc_status_index(0),
		nh("~")
	{}

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		esc_info_pub = nh.advertise<mavros_msgs::ESCInfo>("esc_info", 10);
		esc_status_pub = nh.advertise<mavros_msgs::ESCStatus>("esc_status", 10);

		enable_connection_cb();
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&ESCStatusPlugin::handle_esc_info),
			       make_handler(&ESCStatusPlugin::handle_esc_status),
		};
	}

private:
	using lock_guard = std::lock_guard<std::mutex>;
	std::mutex mutex;

	ros::NodeHandle nh;

	ros::Publisher esc_info_pub;
	ros::Publisher esc_status_pub;
	mavros_msgs::ESCInfo _esc_info;
	mavros_msgs::ESCStatus _esc_status;
	uint8_t _max_esc_count;
	uint8_t _max_esc_info_index;
	uint8_t _max_esc_status_index;
	const uint8_t batch_size = 4;

	void handle_esc_info(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ESC_INFO &esc_info)
	{
		lock_guard lock(mutex);

		_esc_info.header.stamp = m_uas->synchronise_stamp(esc_info.time_usec);

		uint8_t esc_index = esc_info.index;

		_esc_info.counter = esc_info.counter;
		_esc_info.count = esc_info.count;
		_esc_info.connection_type = esc_info.connection_type;
		_esc_info.info = esc_info.info;

		if (_esc_info.count > _max_esc_count)
		{
			_max_esc_count = _esc_info.count;
		}

		if (_esc_info.esc_info.size() < _max_esc_count)
		{
			_esc_info.esc_info.resize(_max_esc_count);
		}

		for (int i = 0; i < std::min<ssize_t>(batch_size, ssize_t(_max_esc_count) - esc_index); i++)
		{
			_esc_info.esc_info[esc_index + i].header = _esc_info.header;
			_esc_info.esc_info[esc_index + i].failure_flags = esc_info.failure_flags[i];
			_esc_info.esc_info[esc_index + i].error_count = esc_info.error_count[i];
			_esc_info.esc_info[esc_index + i].temperature = esc_info.temperature[i];
		}

		_max_esc_info_index = std::max(_max_esc_info_index, esc_info.index);

		if (_max_esc_info_index == esc_info.index)
		{
			esc_info_pub.publish(_esc_info);
		}
	}

	void handle_esc_status(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ESC_STATUS &esc_status)
	{
		lock_guard lock(mutex);

		uint8_t esc_index = esc_status.index;

		if (_esc_status.esc_status.size() < _max_esc_count)
		{
			_esc_status.esc_status.resize(_max_esc_count);
		}

		_esc_status.header.stamp = m_uas->synchronise_stamp(esc_status.time_usec);

		for (int i = 0; i < std::min<ssize_t>(batch_size, ssize_t(_max_esc_count) - esc_index); i++)
		{
			_esc_status.esc_status[esc_index + i].header = _esc_status.header;
			_esc_status.esc_status[esc_index + i].rpm = esc_status.rpm[i];
			_esc_status.esc_status[esc_index + i].voltage = esc_status.voltage[i];
			_esc_status.esc_status[esc_index + i].current = esc_status.current[i];
		}

		_max_esc_status_index = std::max(_max_esc_status_index, esc_status.index);

		if (_max_esc_status_index == esc_status.index)
		{
			esc_status_pub.publish(_esc_status);
		}
	}

	void connection_cb(bool connected) override
	{
		lock_guard lock(mutex);

		_max_esc_count = 0;
		_max_esc_status_index = 0;
		_max_esc_info_index = 0;
		_esc_info.esc_info.resize(0);
		_esc_status.esc_status.resize(0);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ESCStatusPlugin, mavros::plugin::PluginBase)
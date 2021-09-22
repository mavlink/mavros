/**
 * @brief APM ESC Telemetry plugin
 * @file esc_telemetry.cpp
 * @author Braedon O'Meara <braedon@rizse.io>
 * @author Karthik Desai <karthik.desai@iav.de>
 * @author Dr.-Ing. Amilcar do Carmo Lucas <amilcar.lucas@iav.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2021 Braedon O'Meara <braedon@rizse.io>.
 * Copyright 2021 Karthik Desai <karthik.desai@iav.de>.
 * Copyright 2021 Dr.-Ing. Amilcar do Carmo Lucas <amilcar.lucas@iav.de>.
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

		esc_telemetry_pub = nh.advertise<mavros_msgs::ESCTelemetry>("esc_telemetry", 1);

		enable_connection_cb();

		// Read the diagnostic variables
		auto pnh = ros::NodeHandle(ros::NodeHandle("~esc_telemetry"), "diagnostics");
		pnh.param("enabled", _diag_enabled, false);
		if (_diag_enabled)
		{
			ROS_INFO_STREAM("Diagnostics are enabled!!!");
			pnh.param("temp_min/nok", _temp_min_nok, 0.0f);
			pnh.param("temp_min/ok", _temp_min_ok, 1.0f);
			pnh.param("volt_min/nok", _volt_min_nok, 14.0f);
			pnh.param("volt_min/ok", _volt_min_ok, 15.0f);
			pnh.param("curr_min/nok", _curr_min_nok, 4.0f);
			pnh.param("curr_min/ok", _curr_min_ok, 5.0f);
			pnh.param("rpm_min/nok", _rpm_min_nok, 2000);
			pnh.param("rpm_min/ok", _rpm_min_ok, 3000);
			pnh.param("temp_max/nok", _temp_max_nok, 90.0f);
			pnh.param("temp_max/ok", _temp_max_ok, 85.0f);
			pnh.param("volt_max/nok", _volt_max_nok, 17.0f);
			pnh.param("volt_max/ok", _volt_max_ok, 16.0f);
			pnh.param("curr_max/nok", _curr_max_nok, 10.0f);
			pnh.param("curr_max/ok", _curr_max_ok, 8.0f);
			pnh.param("rpm_max/nok", _rpm_max_nok, 12000);
			pnh.param("rpm_max/ok", _rpm_max_ok, 9000);
			pnh.param("count/min_delta", _count_min_delta, 10);
			UAS_DIAG(m_uas).add("ESC temperature", this, &ESCTelemetryPlugin::run_temperature_diagnostics);
			UAS_DIAG(m_uas).add("ESC voltage", this, &ESCTelemetryPlugin::run_voltage_diagnostics);
			UAS_DIAG(m_uas).add("ESC current", this, &ESCTelemetryPlugin::run_current_diagnostics);
			UAS_DIAG(m_uas).add("ESC rpm", this, &ESCTelemetryPlugin::run_rpm_diagnostics);
			UAS_DIAG(m_uas).add("ESC message count", this, &ESCTelemetryPlugin::run_count_diagnostics);
		}
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

	// vars used for diagnostics
	bool _diag_enabled;
	static const std::size_t MAV_NR_ESCS = sizeof(mavros_msgs::ESCTelemetry)/sizeof(mavros_msgs::ESCTelemetryItem);
	bool _temp_min_errors[MAV_NR_ESCS];
	bool _volt_min_errors[MAV_NR_ESCS];
	bool _curr_min_errors[MAV_NR_ESCS];
	bool _rpm_min_errors[MAV_NR_ESCS];
	bool _temp_max_errors[MAV_NR_ESCS];
	bool _volt_max_errors[MAV_NR_ESCS];
	bool _curr_max_errors[MAV_NR_ESCS];
	bool _rpm_max_errors[MAV_NR_ESCS];
	uint16_t _msg_count[MAV_NR_ESCS];
	uint8_t _esc_count = 0;
	// Temperature checks
	float _temp_min_nok;
	float _temp_min_ok;
	float _temp_max_nok;
	float _temp_max_ok;
	// Voltage checks
	float _volt_min_nok;
	float _volt_min_ok;
	float _volt_max_nok;
	float _volt_max_ok;
	// Current checks
	float _curr_min_nok;
	float _curr_min_ok;
	float _curr_max_nok;
	float _curr_max_ok;
	// RPM checks
	int _rpm_min_nok;
	int _rpm_min_ok;
	int _rpm_max_nok;
	int _rpm_max_ok;
	// Count checks, should be uint16_t but ROS only allows int parameters
	int _count_min_delta;
	int _consecutively_detected_delta_errors;

	template <typename msgT>
	void handle_esc_telemetry(const mavlink::mavlink_message_t *msg, msgT &et, size_t offset = 0)
	{
		if (_esc_count == 0) {
			return;
		}
		auto stamp = ros::Time::now();

		_esc_telemetry.header.stamp = stamp;
		for (size_t i = 0; i < et.temperature.size(); i++) {
			auto &p = _esc_telemetry.esc_telemetry.at(offset + i);

			p.header.stamp = stamp;
			p.temperature = et.temperature[i];
			p.voltage = et.voltage[i] * 0.01f;			// centiV -> V
			p.current = et.current[i] * 0.01f;			// centiA -> A
			p.totalcurrent = et.totalcurrent[i] * 0.001f;		// mAh -> Ah
			p.rpm = et.rpm[i];
			p.count = et.count[i];
		}

		esc_telemetry_pub.publish(_esc_telemetry);
	}

	void handle_esc_telemetry_1_to_4(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::ESC_TELEMETRY_1_TO_4 &esc_telemetry)
	{
		lock_guard lock(mutex);
		handle_esc_telemetry(msg, esc_telemetry, 0);
	}

	void handle_esc_telemetry_5_to_8(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::ESC_TELEMETRY_5_TO_8 &esc_telemetry)
	{
		lock_guard lock(mutex);
		handle_esc_telemetry(msg, esc_telemetry, 4);
	}

	void handle_esc_telemetry_9_to_12(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::ESC_TELEMETRY_9_TO_12 &esc_telemetry)
	{
		lock_guard lock(mutex);
		handle_esc_telemetry(msg, esc_telemetry, 8);
	}

	void set_esc_count(mavlink::minimal::MAV_TYPE type)
	{
		switch (type)
		{
			case mavlink::minimal::MAV_TYPE::TRICOPTER:
				_esc_count = 3;
				break;
			case mavlink::minimal::MAV_TYPE::QUADROTOR:
				_esc_count = 4;
				break;
			case mavlink::minimal::MAV_TYPE::HEXAROTOR:
				_esc_count = 6;
				break;
			case mavlink::minimal::MAV_TYPE::OCTOROTOR:
				_esc_count = 8;
				break;
			case mavlink::minimal::MAV_TYPE::DECAROTOR:
				_esc_count = 10;
				break;
			case mavlink::minimal::MAV_TYPE::DODECAROTOR:
				_esc_count = 12;
				break;
			default:
				_esc_count = 0;
		}
		if (_esc_count > MAV_NR_ESCS)
		{
			_esc_count = MAV_NR_ESCS;
		}
		ROS_INFO("%d ESCs detected", _esc_count);
	}

	void connection_cb(bool connected) override
	{
		lock_guard lock(mutex);

		if (connected)
		{
			set_esc_count(m_uas->get_type());
		}
		else
		{
			_esc_count = 0;
		}
	}

	template <typename field_type, typename data_type>
	void run_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat, field_type field, bool (&min_errors)[MAV_NR_ESCS], bool (&max_errors)[MAV_NR_ESCS],
						 const char* measurement, const char* units, const data_type min_ok, const data_type min_nok, const data_type max_ok, const data_type max_nok)
	{
		std::string diag_str = measurement;
		diag_str += " ";
		diag_str += units;
		// validate user-defined treshold parameters
		if (min_nok > min_ok)
		{
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, diag_str + " minimum nok threshold (" + std::to_string(min_nok) + ") must be smaller than minumum ok threshold (" + std::to_string(min_ok) + ") parameter");
			return;
		}
		if (min_ok > max_ok)
		{
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, diag_str + " minimum ok threshold (" + std::to_string(min_ok) + ") must be smaller than maximum ok threshold (" + std::to_string(max_ok) + ") parameter");
			return;
		}
		if (max_ok > max_nok)
		{
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, diag_str + " maximum ok threshold (" + std::to_string(max_ok) + ") must be smaller than maximum nok threshold (" + std::to_string(max_nok) + ") parameter");
			return;
		}
		// display thresholds in ascending order (user friendly way)
		stat.add("Min NOk " + diag_str, min_nok);
		stat.add("Min Ok  " + diag_str, min_ok);
		stat.add("Max Ok  " + diag_str, max_ok);
		stat.add("Max NOk " + diag_str, max_nok);

		// test if each value is within the interval defined by the thresholds
		diag_str += " of ESC ";
		std::string error_str;
		bool error = false;
		{
			lock_guard lock(mutex);
			for (int i = 0; i < _esc_count; i++)
			{
				bool found_error = false;
				mavros_msgs::ESCTelemetryItem t = _esc_telemetry.esc_telemetry[i];
				stat.add(diag_str + std::to_string(i+1), t.*field);

				if (min_errors[i] == false) // in the past the measurement value was above min valid threshold
				{
					if (t.*field < min_nok)
					{
						min_errors[i] = true;
						found_error = true;
					}
				}
				else
				{
					if (t.*field > min_ok)
					{
						min_errors[i] = false;
					}
					else
					{
						found_error = true;
					}
				}

				if (max_errors[i] == false) // in the past the measurement value was bellow max valid threshold
				{
					if (t.*field > max_nok)
					{
						max_errors[i] = true;
						found_error = true;
					}
				}
				else
				{
					if (t.*field < max_ok)
					{
						max_errors[i] = false;
					}
					else
					{
						found_error = true;
					}
				}
				if (found_error)
				{
					error_str += (std::to_string(i+1) + " " + std::to_string(t.*field) + ", ");
					error = true;
				}

			}
		}

		// summarize the results
		if (_esc_count == 0)
		{
			stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, "N/A");
		}
		else
		{
			if (error)
			{
				stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Errors on " + diag_str + error_str);
			}
			else
			{
				stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Normal");
			}
		}
	}

	void run_temperature_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		run_diagnostics(stat, &mavros_msgs::ESCTelemetryItem::temperature, _temp_min_errors, _temp_max_errors, "temperature", "(degC)", _temp_min_ok, _temp_min_nok, _temp_max_ok, _temp_max_nok);
	}

	void run_voltage_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		run_diagnostics(stat, &mavros_msgs::ESCTelemetryItem::voltage, _volt_min_errors, _volt_max_errors, "voltage", "(V)", _volt_min_ok, _volt_min_nok, _volt_max_ok, _volt_max_nok);
	}

	void run_current_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		run_diagnostics(stat, &mavros_msgs::ESCTelemetryItem::current, _curr_min_errors, _curr_max_errors, "current", "(A)", _curr_min_ok, _curr_min_nok, _curr_max_ok, _curr_max_nok);
	}

	void run_rpm_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		run_diagnostics(stat, &mavros_msgs::ESCTelemetryItem::rpm, _rpm_min_errors, _rpm_max_errors, "RPM", "(1/min)", _rpm_min_ok, _rpm_min_nok, _rpm_max_ok, _rpm_max_nok);
	}

	void run_count_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		stat.add("Min Count delta", _count_min_delta);
		std::string error_str;
		bool error = false;
		{
			lock_guard lock(mutex);
			for (int i = 0; i < _esc_count; i++)
			{
				mavros_msgs::ESCTelemetryItem t = _esc_telemetry.esc_telemetry[i];
				stat.add("Count reported by ESC " + std::to_string(i+1), t.count);

				uint16_t count_delta = t.count - _msg_count[i];
				if (count_delta < uint16_t(_count_min_delta))
				{
					error = true;
					error_str += (std::to_string(i+1) + " " + std::to_string(count_delta) + ", ");
				}
				_msg_count[i] = t.count;
			}
		}
		if (_esc_count == 0)
		{
			stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, "N/A");
		}
		else
		{
			if ((_consecutively_detected_delta_errors > 1) && error)
			{
				stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Count issues reported by ESCs " + error_str);
			}
			else
			{
				stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Normal");
			}
		}
		if (error) {
			if (_consecutively_detected_delta_errors < INT32_MAX-1) {
				_consecutively_detected_delta_errors++;
			}
		} else {
			_consecutively_detected_delta_errors = 0;
		}
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ESCTelemetryPlugin, mavros::plugin::PluginBase)

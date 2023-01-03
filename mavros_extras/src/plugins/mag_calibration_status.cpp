/**
 * @brief MagCalStatus plugin
 * @file MagCalStatus.cpp
 * @author André Ferreira <andre.ferreira@beyond-vision.pt>
 *
 * @example MagCalStatus.cpp
 * @addtogroup plugin
 * @{
 */

#include <mavros/mavros_plugin.h>
#include <std_msgs/UInt8.h>
#include <mavros_msgs/MagnetometerReporter.h>
namespace mavros {
namespace std_plugins {
/**
 * @brief MagCalStatus plugin.
 *
 * Example and "how to" for users.
 */
class MagCalStatusPlugin : public plugin::PluginBase {
public:
	MagCalStatusPlugin() : PluginBase(),
		mcs_nh("~mag_calibration")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		mcs_pub = mcs_nh.advertise<std_msgs::UInt8>("status", 2, true);
		mcr_pub = mcs_nh.advertise<mavros_msgs::MagnetometerReporter>("report", 2, true);
	}

	/**
	 * This function returns message subscriptions.
	 *
	 * Each subscription made by PluginBase::make_handler() template.
	 * Two variations:
	 *  - With automatic decoding and framing error filtering (see handle_heartbeat)
	 *  - Raw message with framig status (see handle_systemtext)
	 */
	Subscriptions get_subscriptions() {
		return {
			/* automatic message deduction by second argument */
			make_handler(&MagCalStatusPlugin::handle_status),
			make_handler(&MagCalStatusPlugin::handle_report),
		};
	}

private:
	ros::NodeHandle mcs_nh;
	ros::Publisher mcs_pub;
	ros::Publisher mcr_pub;
	std::array<bool, 8> calibration_show;
	std::array<uint8_t, 8> _rg_compass_cal_progress;
	//Send progress of magnetometer calibration
	void handle_status(const mavlink::mavlink_message_t *, mavlink::ardupilotmega::msg::MAG_CAL_PROGRESS &mp) {
		auto mcs = boost::make_shared<std_msgs::UInt8>();

		// How many compasses are we calibrating?
		std::bitset<8> compass_calibrating = mp.cal_mask;

		if (compass_calibrating[mp.compass_id]) {
			// Each compass gets a portion of the overall progress
			if (mp.completion_pct < 95) {
				calibration_show[mp.compass_id] = true;
			}
			_rg_compass_cal_progress[mp.compass_id] = mp.completion_pct;
		}

		// Prevent data over 100% after cal_mask reset bit assigned to compass_id
		uint16_t total_percentage = 0;
		for (size_t i = 0; i < 8 && (compass_calibrating >> i).any(); i++) {
			if (compass_calibrating[i]) {
				total_percentage += static_cast<uint8_t>(_rg_compass_cal_progress[i]);
			}
		}

		mcs->data = total_percentage / compass_calibrating.count();

		mcs_pub.publish(mcs);
	}

	//Send report after calibration is done
	void handle_report(const mavlink::mavlink_message_t *, mavlink::common::msg::MAG_CAL_REPORT &mr) {
		if (calibration_show[mr.compass_id]) {
			auto mcr = boost::make_shared<mavros_msgs::MagnetometerReporter>();
			mcr->header.stamp = ros::Time::now();
			mcr->header.frame_id = std::to_string(mr.compass_id);
			mcr->report = mr.cal_status;
			mcr->confidence = mr.orientation_confidence;
			mcr_pub.publish(mcr);
			calibration_show[mr.compass_id] = false;
		}
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::MagCalStatusPlugin, mavros::plugin::PluginBase)

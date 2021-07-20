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
		mcs_nh("~MagCalibration")
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
	uint8_t _rgCompassCalProgress[3] = {0};
	bool calibration[3]={true, true, true};
	//Send progress of magnetometer calibration
	void handle_status(const mavlink::mavlink_message_t *, mavlink::ardupilotmega::msg::MAG_CAL_PROGRESS &mp) {
		auto mcs = boost::make_shared<std_msgs::UInt8>();

		// How many compasses are we calibrating?
		int compassCalCount = 0;
		for (int i=0; i<3; i++) {
			if (mp.cal_mask & (1 << i)) {
				compassCalCount++;
			}
		}

		if (mp.compass_id < 3 && compassCalCount != 0) {
			// Each compass gets a portion of the overall progress
			_rgCompassCalProgress[mp.compass_id] = mp.completion_pct;
		}

		mcs->data = static_cast<uint8_t>((_rgCompassCalProgress[0] + _rgCompassCalProgress[1] + _rgCompassCalProgress[2])/compassCalCount);

		mcs_pub.publish(mcs);
	}

	//Send report after calibration is done
	void handle_report(const mavlink::mavlink_message_t *, mavlink::common::msg::MAG_CAL_REPORT &mr) {
		if(calibration[mr.compass_id]) {
			auto mcr = boost::make_shared<mavros_msgs::MagnetometerReporter>();
			mcr->header.stamp = ros::Time::now();
			mcr->header.frame_id = std::to_string(mr.compass_id);
			mcr->report = mr.cal_status;
			mcr->confidence = mr.orientation_confidence;
			mcr_pub.publish(mcr);
			calibration[mr.compass_id] = false;
		}
	}
};

}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::MagCalStatusPlugin, mavros::plugin::PluginBase)

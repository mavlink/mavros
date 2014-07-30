/**
 * @brief PX4Flow pub plugin
 * @file px4flow.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_extras/OpticalFlow.h>

namespace mavplugin {

class PX4FlowPlugin : public MavRosPlugin {
public:
	PX4FlowPlugin()
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;
		flow_pub = nh.advertise<mavros_extras::OpticalFlow>("optical_flow", 10);
	}

	const std::string get_name() const {
		return "PX4Flow";
	}

	const std::vector<uint8_t> get_supported_messages() const {
		return {
			MAVLINK_MSG_ID_OPTICAL_FLOW
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		handle_flow(msg);
	}

private:
	UAS *uas;

	ros::Publisher flow_pub;

	void handle_flow(const mavlink_message_t *msg) {
		if (flow_pub.getNumSubscribers() == 0)
			return;

		mavlink_optical_flow_t flow;
		mavlink_msg_optical_flow_decode(msg, &flow);

		mavros_extras::OpticalFlowPtr flow_msg =
			boost::make_shared<mavros_extras::OpticalFlow>();

		flow_msg->flow_x = flow.flow_x;
		flow_msg->flow_y = flow.flow_y;
		flow_msg->flow_comp_m_x	 = flow.flow_comp_m_x;
		flow_msg->flow_comp_m_y	 = flow.flow_comp_m_y;
		flow_msg->quality = flow.quality;
		flow_msg->ground_distance = flow.ground_distance;

		//TODO header
		flow_pub.publish(flow_msg);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::PX4FlowPlugin, mavplugin::MavRosPlugin)

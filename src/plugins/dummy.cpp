/** dummy plugin */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

namespace mavplugin {

class DummyPlugin : public MavRosPlugin {
public:
	DummyPlugin()
	{
		std::cout << "dummy constructor" << std::endl;
	};

	void initialize(ros::NodeHandle &nh,
			const boost::shared_ptr<mavconn::MAVConnInterface> &mav_link,
			diagnostic_updater::Updater &diag_updater)
	{
		std::cout << "initialize" << std::endl;
	};

	std::string get_name()
	{
		return "Dummy";
	};

	std::vector<uint8_t> get_supported_messages()
	{
		return {0, 1, 2, 3, 4};
	};

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid)
	{
		std::cout << "Dummy::message_rx_cb" << std::endl;
	};
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::DummyPlugin, mavplugin::MavRosPlugin)


#include <mavros/mavros_plugin.h>
#include <mavros_msgs/PlayTuneV2.h>
#include <cstring>

namespace mavros
{
namespace extra_plugins
{
class PlayTunePlugin : public plugin::PluginBase
{
public:
	PlayTunePlugin() : PluginBase(), nh("~") {}

	void initialize(UAS& uas_) override
	{
		PluginBase::initialize(uas_);
		sub = nh.subscribe("play_tune", 1, &PlayTunePlugin::callback, this);
	}

	Subscriptions get_subscriptions() override {
		return { /* No subscriptions */ };
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber sub;

	void callback(const mavros_msgs::PlayTuneV2::ConstPtr& tune)
	{
		auto msg = mavlink::common::msg::PLAY_TUNE_V2{};
		m_uas->msg_set_target(msg);
		msg.format = tune->format;
		mavlink::set_string_z(msg.tune, tune->tune);
		UAS_FCU(m_uas)->send_message_ignore_drop(msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::PlayTunePlugin, mavros::plugin::PluginBase)

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/ESCTelemetryItem.h>
#include <mavros_msgs/ESCTelemetry.h>

namespace mavros
{
namespace extra_plugins
{
/**
 * @brief ESC status plugin
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

        esc_telemetry_1_to_4_pub = nh.advertise<mavros_msgs::ESCTelemetry>("esc_telemetry_1_to_4", 10);
        esc_telemetry_5_to_8_pub = nh.advertise<mavros_msgs::ESCTelemetry>("esc_telemetry_5_to_8", 10);
        esc_telemetry_9_to_12_pub = nh.advertise<mavros_msgs::ESCTelemetry>("esc_telemetry_9_to_12", 10);

        //set the sizes of each of the telemetry messages
        _esc_telemetry_1_to_4.esc_telemetry.resize(4);
        _esc_telemetry_5_to_8.esc_telemetry.resize(4);
        _esc_telemetry_9_to_12.esc_telemetry.resize(4);

        enable_connection_cb();
    }

    Subscriptions get_subscriptions() override
    {
        return {
            make_handler(&ESCTelemetryPlugin::handle_esc_telemetry_1_to_4),
            /*make_handler(&ESCTelemetryPlugin::handle_esc_telemetry_5_to_8),
            make_handler(&ESCTelemetryPlugin::handle_esc_telemetry_9_to_12),*/
        };
    }

private:

    using lock_guard = std::lock_guard<std::mutex>;
    std::mutex mutex;

    ros::NodeHandle nh;

    ros::Publisher esc_telemetry_1_to_4_pub;
    ros::Publisher esc_telemetry_5_to_8_pub;
    ros::Publisher esc_telemetry_9_to_12_pub;
    mavros_msgs::ESCTelemetry _esc_telemetry_1_to_4;
    mavros_msgs::ESCTelemetry _esc_telemetry_5_to_8;
    mavros_msgs::ESCTelemetry _esc_telemetry_9_to_12;

    void handle_esc_telemetry_1_to_4(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::ESC_TELEMETRY_1_TO_4 &esc_telemetry)
    {
        lock_guard lock(mutex);

        for (int i = 0; i < 4; i++)
        {
            _esc_telemetry_1_to_4.esc_telemetry[i].temperature = esc_telemetry.temperature[i];
            _esc_telemetry_1_to_4.esc_telemetry[i].voltage = esc_telemetry.voltage[i];
            _esc_telemetry_1_to_4.esc_telemetry[i].current = esc_telemetry.current[i];
            _esc_telemetry_1_to_4.esc_telemetry[i].totalcurrent = esc_telemetry.totalcurrent[i];
            _esc_telemetry_1_to_4.esc_telemetry[i].rpm = esc_telemetry.rpm[i];
            _esc_telemetry_1_to_4.esc_telemetry[i].count = esc_telemetry.count[i];
        }
        
        esc_telemetry_1_to_4_pub.publish(_esc_telemetry_1_to_4);

    }

    void handle_esc_telemetry_5_to_8(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::ESC_TELEMETRY_5_TO_8 &esc_telemetry)
    {
        lock_guard lock(mutex);

        for (int i = 0; i < 4; i++)
        {
            _esc_telemetry_5_to_8.esc_telemetry[i].temperature = esc_telemetry.temperature[i];
            _esc_telemetry_5_to_8.esc_telemetry[i].voltage = esc_telemetry.voltage[i];
            _esc_telemetry_5_to_8.esc_telemetry[i].current = esc_telemetry.current[i];
            _esc_telemetry_5_to_8.esc_telemetry[i].totalcurrent = esc_telemetry.totalcurrent[i];
            _esc_telemetry_5_to_8.esc_telemetry[i].rpm = esc_telemetry.rpm[i];
            _esc_telemetry_5_to_8.esc_telemetry[i].count = esc_telemetry.count[i];
        }
        
        esc_telemetry_5_to_8_pub.publish(_esc_telemetry_5_to_8);

    }

    void handle_esc_telemetry_9_to_12(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::ESC_TELEMETRY_9_TO_12 &esc_telemetry)
    {
        lock_guard lock(mutex);

        for (int i = 0; i < 4; i++)
        {
            _esc_telemetry_9_to_12.esc_telemetry[i].temperature = esc_telemetry.temperature[i];
            _esc_telemetry_9_to_12.esc_telemetry[i].voltage = esc_telemetry.voltage[i];
            _esc_telemetry_9_to_12.esc_telemetry[i].current = esc_telemetry.current[i];
            _esc_telemetry_9_to_12.esc_telemetry[i].totalcurrent = esc_telemetry.totalcurrent[i];
            _esc_telemetry_9_to_12.esc_telemetry[i].rpm = esc_telemetry.rpm[i];
            _esc_telemetry_9_to_12.esc_telemetry[i].count = esc_telemetry.count[i];
        }
        
        esc_telemetry_9_to_12_pub.publish(_esc_telemetry_9_to_12);

    }

    void connection_cb(bool connected) override
	{
        //NOOP
	}

};
}   // namespace extra_plugins
}   // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ESCTelemetryPlugin, mavros::plugin::PluginBase)
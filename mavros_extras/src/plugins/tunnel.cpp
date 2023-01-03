#include <algorithm>
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/Tunnel.h>
#include <stdexcept>

namespace mavros
{
namespace extra_plugins
{
class TunnelPlugin : public plugin::PluginBase
{
  public:
    TunnelPlugin() : PluginBase(), nh_("~tunnel") {}

    void initialize(UAS& uas_) override
    {
        PluginBase::initialize(uas_);
        sub_ = nh_.subscribe("in", 10, &TunnelPlugin::ros_callback, this);
        pub_ = nh_.advertise<mavros_msgs::Tunnel>("out", 10);
    }

    Subscriptions get_subscriptions() override
    {
        return {make_handler(&TunnelPlugin::mav_callback)};
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    void ros_callback(const mavros_msgs::Tunnel::ConstPtr& ros_tunnel)
    {
        try
        {
            const auto mav_tunnel =
                copy_tunnel<mavros_msgs::Tunnel, mavlink::common::msg::TUNNEL>(
                    *ros_tunnel);

            UAS_FCU(m_uas)->send_message_ignore_drop(mav_tunnel);
        }
        catch (const std::overflow_error& e)
        {
            ROS_ERROR_STREAM_NAMED("tunnel", e.what());
        }
    }

    void mav_callback(const mavlink::mavlink_message_t*,
                      mavlink::common::msg::TUNNEL& mav_tunnel)
    {
        try
        {
            const auto ros_tunnel =
                copy_tunnel<mavlink::common::msg::TUNNEL, mavros_msgs::Tunnel>(
                    mav_tunnel);

            pub_.publish(ros_tunnel);
        }
        catch (const std::overflow_error& e)
        {
            ROS_ERROR_STREAM_NAMED("tunnel", e.what());
        }
    }

    template <typename From, typename To>
    static To copy_tunnel(const From& from) noexcept(false)
    {
        static constexpr auto max_payload_length =
            sizeof(mavlink::common::msg::TUNNEL::payload) /
            sizeof(mavlink::common::msg::TUNNEL::payload[0]);

        if (from.payload_length > max_payload_length)
        {
            throw std::overflow_error("too long payload length");
        }

        auto to = To{};

        to.target_system = from.target_system;
        to.target_component = from.target_component;
        to.payload_type = from.payload_type;
        to.payload_length = from.payload_length;
        std::copy(from.payload.begin(),
                  from.payload.begin() + from.payload_length,
                  to.payload.begin());

        return to;
    }
};
}  // namespace extra_plugins
}  // namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::TunnelPlugin,
                       mavros::plugin::PluginBase)

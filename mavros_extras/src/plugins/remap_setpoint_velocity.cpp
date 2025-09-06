#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <thread>
#include <atomic>
#include <mutex>

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;
using mavlink::common::MAV_FRAME;

/**
 * @brief Custom Setpoint velocity plugin
 * @plugin custom_setpoint_velocity
 *
 * Send setpoint velocities to FCU controller via individual Float32 topics.
 */
class CustomSetpointVelocityPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<CustomSetpointVelocityPlugin>
{
public:
  explicit CustomSetpointVelocityPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "custom_setpoint_velocity"),
    cur_vel_x(0.0f),
    cur_vel_y(0.0f), 
    cur_vel_z(0.0f),
    cur_vel_r(0.0f),
    run_publisher_(true),
    active_(false)
  {
    enable_node_watch_parameters();

    auto sensor_qos = rclcpp::SensorDataQoS();

    twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
      
    vel_x_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "~/vel_x", sensor_qos, 
      std::bind(&CustomSetpointVelocityPlugin::setpoint_vel_x_cb, this, _1));

    vel_y_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "~/vel_y", sensor_qos,
      std::bind(&CustomSetpointVelocityPlugin::setpoint_vel_y_cb, this, _1));

    vel_z_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "~/vel_z", sensor_qos,
      std::bind(&CustomSetpointVelocityPlugin::setpoint_vel_z_cb, this, _1));

    vel_r_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "~/vel_r", sensor_qos,
      std::bind(&CustomSetpointVelocityPlugin::setpoint_vel_r_cb, this, _1));

    // Timer
    timeout_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { check_timeout(); });
    last_msg_time_ = node->now();

    // Start publisher thread
    publisher_thread_ = std::thread(&CustomSetpointVelocityPlugin::publish_loop, this);
  }

  ~CustomSetpointVelocityPlugin()
  {
    run_publisher_.store(false);
    if (publisher_thread_.joinable()) {
      publisher_thread_.join();
    }
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetPositionTargetLocalNEDMixin<CustomSetpointVelocityPlugin>;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_x_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_y_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_z_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_r_sub_;
  
  std::atomic<float> cur_vel_x;
  std::atomic<float> cur_vel_y;
  std::atomic<float> cur_vel_z;
  std::atomic<float> cur_vel_r;

  std::thread publisher_thread_;
  std::atomic<bool> run_publisher_;
  std::atomic<bool> active_;
  
  // Timeout management (protected by mutex for thread safety)
  std::mutex time_mutex_;
  rclcpp::Time last_msg_time_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  void publish_loop()
  {
    rclcpp::Rate rate(50); // 50 Hz publishing rate
    
    while (rclcpp::ok() && run_publisher_.load(std::memory_order_acquire))
    {
      if (active_.load(std::memory_order_acquire)) {
        send_setpoint_velocity();
      }
      rate.sleep();
    }
  }

  /**
   * @brief Send combined velocity setpoint to MAVROS
   * 
   * @warning Send only VX VY VZ and RZ
   */
  void send_setpoint_velocity()
  {
    geometry_msgs::msg::Twist twist;

    // Read current velocities atomically
    twist.linear.x = cur_vel_x.load(std::memory_order_acquire);
    twist.linear.y = cur_vel_y.load(std::memory_order_acquire);
    twist.linear.z = cur_vel_z.load(std::memory_order_acquire);
    twist.angular.z = cur_vel_r.load(std::memory_order_acquire);

    twist_pub_->publish(twist);
  }

  void check_timeout()
  {
    if (!active_.load(std::memory_order_acquire))
      return;

    rclcpp::Time current_time;
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      current_time = last_msg_time_;
    }

    // Check if more than 1 second since last message
    if ((node->now() - current_time).seconds() > 1.0) {
      RCLCPP_WARN(get_logger(), "Velocity setpoint timeout - stopping");
      
      // Zero all velocities
      cur_vel_x.store(0.0f, std::memory_order_release);
      cur_vel_y.store(0.0f, std::memory_order_release);
      cur_vel_z.store(0.0f, std::memory_order_release);
      cur_vel_r.store(0.0f, std::memory_order_release);
      send_setpoint_velocity();

      // Deactivate publishing
      active_.store(false, std::memory_order_release);
    }
  }

  void update_timestamp_and_activate()
  {
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      last_msg_time_ = node->now();
    }
    active_.store(true, std::memory_order_release);
  }

  /* -*- callbacks -*- */
  void setpoint_vel_x_cb(const std_msgs::msg::Float32::SharedPtr vel_x)
  {
    cur_vel_x.store(vel_x->data, std::memory_order_release);
    update_timestamp_and_activate();
  }

  void setpoint_vel_y_cb(const std_msgs::msg::Float32::SharedPtr vel_y)
  {
    cur_vel_y.store(vel_y->data, std::memory_order_release);
    update_timestamp_and_activate();
  }

  void setpoint_vel_z_cb(const std_msgs::msg::Float32::SharedPtr vel_z)
  {
    cur_vel_z.store(vel_z->data, std::memory_order_release);
    update_timestamp_and_activate();
  }

  void setpoint_vel_r_cb(const std_msgs::msg::Float32::SharedPtr vel_r)
  {
    cur_vel_r.store(vel_r->data, std::memory_order_release);
    update_timestamp_and_activate();
  }
};

} // namespace std_plugins
} // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::CustomSetpointVelocityPlugin)
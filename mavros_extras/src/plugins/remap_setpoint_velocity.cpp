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
namespace extra_plugins
{
using namespace std::placeholders;
using mavlink::common::MAV_FRAME;

/**
 * @brief Custom Setpoint velocity plugin
 * @plugin custom_setpoint_velocity
 *
 * Send setpoint velocities to FCU controller via individual Float32 topics.
 */
class RemapSetpointVelocityPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<RemapSetpointVelocityPlugin>
{
public:
  explicit RemapSetpointVelocityPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "remap_setpoint_velocity"),
    cur_vel_x(0.0f),
    cur_vel_y(0.0f), 
    cur_vel_z(0.0f),
    cur_vel_r(0.0f),
    run_publisher_(true),
    active_(false),
    COMPONENT_TIMEOUT_(0.2)
  {
    enable_node_watch_parameters();

    auto sensor_qos = rclcpp::SensorDataQoS();

    twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
      
    vel_x_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "~/vel_x", sensor_qos, 
      std::bind(&RemapSetpointVelocityPlugin::setpoint_vel_x_cb, this, _1));

    vel_y_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "~/vel_y", sensor_qos,
      std::bind(&RemapSetpointVelocityPlugin::setpoint_vel_y_cb, this, _1));

    vel_z_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "~/vel_z", sensor_qos,
      std::bind(&RemapSetpointVelocityPlugin::setpoint_vel_z_cb, this, _1));

    vel_r_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "~/vel_r", sensor_qos,
      std::bind(&RemapSetpointVelocityPlugin::setpoint_vel_r_cb, this, _1));

    // Timer
    timeout_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { check_timeout(); });

    auto now = node->now();
    last_vel_x_time_ = now;
    last_vel_y_time_ = now;
    last_vel_z_time_ = now;
    last_vel_r_time_ = now;

    // Start publisher thread
    publisher_thread_ = std::thread(&RemapSetpointVelocityPlugin::publish_loop, this);
  }

  ~RemapSetpointVelocityPlugin()
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
  friend class plugin::SetPositionTargetLocalNEDMixin<RemapSetpointVelocityPlugin>;

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
  rclcpp::Time last_vel_x_time_;
  rclcpp::Time last_vel_y_time_;
  rclcpp::Time last_vel_z_time_;
  rclcpp::Time last_vel_r_time_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  const double COMPONENT_TIMEOUT_;

  void publish_loop()
  {
    rclcpp::Rate rate(40); // 40 Hz publishing rate
    
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

    twist.linear.x = cur_vel_x.load(std::memory_order_acquire);
    twist.linear.y = cur_vel_y.load(std::memory_order_acquire);
    twist.linear.z = cur_vel_z.load(std::memory_order_acquire);
    twist.angular.z = cur_vel_r.load(std::memory_order_acquire);

    twist_pub_->publish(twist);
  }

  void check_timeout()
  {
    // Timeout for each axis
    // If every axis receives no message, then stops publishing
    if (!active_.load(std::memory_order_acquire))
      return;

    rclcpp::Time current_time = node->now();
    bool any_active = false;
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      
      if ((current_time - last_vel_x_time_).seconds() > COMPONENT_TIMEOUT_) {
        cur_vel_x.store(0.0f, std::memory_order_release);
      } else {
        any_active = true;
      }
      
      if ((current_time - last_vel_y_time_).seconds() > COMPONENT_TIMEOUT_) {
        cur_vel_y.store(0.0f, std::memory_order_release);
      } else {
        any_active = true;
      }
      
      if ((current_time - last_vel_z_time_).seconds() > COMPONENT_TIMEOUT_) {
        cur_vel_z.store(0.0f, std::memory_order_release);
      } else {
        any_active = true;
      }
      
      if ((current_time - last_vel_r_time_).seconds() > COMPONENT_TIMEOUT_) {
        cur_vel_r.store(0.0f, std::memory_order_release);
      } else {
        any_active = true;
      }
    }

    if (!any_active) {
      RCLCPP_WARN(get_logger(), "All velocity components timed out - stopping");
      std::cout << "Stopped publishing" << std::endl;
      active_.store(false, std::memory_order_release);
    }
  }

  /* -*- callbacks -*- */
  void setpoint_vel_x_cb(const std_msgs::msg::Float32::SharedPtr vel_x)
  {
    cur_vel_x.store(vel_x->data, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      last_vel_x_time_ = node->now();
    }
    active_.store(true, std::memory_order_release);
  }

  void setpoint_vel_y_cb(const std_msgs::msg::Float32::SharedPtr vel_y)
  {
    cur_vel_y.store(vel_y->data, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      last_vel_y_time_ = node->now();
    }
    active_.store(true, std::memory_order_release);
  }

  void setpoint_vel_z_cb(const std_msgs::msg::Float32::SharedPtr vel_z)
  {
    cur_vel_z.store(vel_z->data, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      last_vel_z_time_ = node->now();
    }
    active_.store(true, std::memory_order_release);
  }

  void setpoint_vel_r_cb(const std_msgs::msg::Float32::SharedPtr vel_r)
  {
    cur_vel_r.store(vel_r->data, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      last_vel_r_time_ = node->now();
    }
    active_.store(true, std::memory_order_release);
  }
};

} // namespace extra_plugins
} // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::RemapSetpointVelocityPlugin)
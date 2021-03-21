/**
 * @brief MavRos node implementation class
 * @file mavros_router.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 *  @brief MAVROS node implementation
 */
/*
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#ifndef MAVROS_MAVROS_ROUTER_HPP_
#define MAVROS_MAVROS_ROUTER_HPP_

#include <array>
#include <mavconn/interface.hpp>
#include <mavconn/mavlink_dialect.hpp>
#include <memory>
//include <mavros/mavlink_diag.h>
#include <mavros/utils.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>

#include <mavros_msgs/msg/mavlink.hpp>
#include <mavros_msgs/srv/endpoint_add.hpp>
#include <mavros_msgs/srv/endpoint_del.hpp>

namespace mavros
{
namespace router
{

using id_t = uint32_t;
using addr_t = uint32_t;

using mavconn::Framing;
using ::mavlink::mavlink_message_t;
using ::mavlink::msgid_t;

using namespace std::placeholders;
using namespace std::chrono_literals;

class Router;

/**
 * Endpoint base class
 *
 * Represents one network connection to the Router
 *
 * One endpoint could map to several remote devices,
 * e.g. mesh radio for swarm.
 */
class Endpoint : public std::enable_shared_from_this<Endpoint>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Endpoint);

  enum class Type
  {
    fcu = 0,
    gcs = 1,
    uas = 2,
  };

  Endpoint()
  : parent(),
    id(0),
    link_type(Type::fcu),
    url{},
    remote_addrs{},
    stale_addrs{}
  {
    const addr_t broadcase_addr = 0;

    // Accept broadcasts by default
    remote_addrs.emplace(broadcase_addr);
  }

  std::shared_ptr<Router> parent;

  uint32_t id;                         // id of the endpoint
  Type link_type;                      // class of the endpoint
  std::string url;                     // url to open that endpoint
  std::set<addr_t> remote_addrs;       // remotes that we heard there
  std::set<addr_t> stale_addrs;        // temporary storage for stale remote addrs

  virtual bool is_open() = 0;
  virtual std::pair<bool, std::string> open() = 0;
  virtual void close() = 0;

  virtual void send_message(const mavlink_message_t * msg, const Framing framing = Framing::ok) = 0;
  virtual void recv_message(const mavlink_message_t * msg, const Framing framing = Framing::ok);
};

/**
 * Router class implements MAVROS Router node
 *
 * Router is essential part that connects several MAVLink devices together.
 * In general there are three device classes that router uses:
 * - a FCU endpoint - should be faced to autopilot firmware(s),
 * - a GCS endpoint - that ep connects control software
 * - a UAS endpoint - special type of endpoint that is used to connect MAVROS UAS node(s) to FCU(s)
 *
 * Some traffic rules:
 * 1. FCU broadcast -> GCS, UAS, but not to any other FCU
 * 2. FCU targeted system/component -> GCS/UAS endpoint that have matching address
 * 3. FCU targeted system -> same as for p.2
 * 4. GCS broadcast -> FCU, UAS
 * 5. GCS targeted -> FCU/UAS maching addr
 * 6. UAS broadcast -> FCU, GCS
 * 7. UAS targeted -> FCU/GCS
 */
class Router : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Router);

  using StrV = std::vector<std::string>;

  Router(std::string node_name = "mavros_router")
  : rclcpp::Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)),
    endpoints{}, stat_msg_sent(0), stat_msg_routed(0), stat_msg_dropped(0)
  {
    RCLCPP_DEBUG(this->get_logger(), "Start mavros::router::Router initialization...");

    set_parameters_handle_ptr =
      this->add_on_set_parameters_callback(std::bind(&Router::on_set_parameters_cb, this, _1));
    this->declare_parameter<StrV>("fcu_urls", StrV());
    this->declare_parameter<StrV>("gcs_urls", StrV());
    this->declare_parameter<StrV>("uas_urls", StrV());

    add_service = this->create_service<mavros_msgs::srv::EndpointAdd>(
      "~/add_endpoint",
      std::bind(&Router::add_endpoint, this, _1, _2));
    del_service = this->create_service<mavros_msgs::srv::EndpointDel>(
      "~/del_endpoint",
      std::bind(&Router::del_endpoint, this, _1, _2));

    // try to reconnect endpoint each 30 seconds
    reconnect_timer =
      this->create_wall_timer(30s, std::bind(&Router::periodic_reconnect_endpoints, this));

    // collect garbage addrs each minute
    stale_addrs_timer =
      this->create_wall_timer(60s, std::bind(&Router::periodic_clear_stale_remote_addrs, this));
  }

  void route_message(Endpoint::SharedPtr src, const mavlink_message_t * msg, const Framing framing);

private:
  friend class Endpoint;

  static std::atomic<id_t> id_counter;

  std::shared_timed_mutex mu;

  // map stores all routing endpoints
  std::unordered_map<id_t, Endpoint::SharedPtr> endpoints;

  std::atomic<size_t> stat_msg_routed;  // amount of messages came to route_messages()
  std::atomic<size_t> stat_msg_sent;    // amount of messages sent
  std::atomic<size_t> stat_msg_dropped; // amount of messages dropped (because there are no suitable endpoint)

  rclcpp::Service<mavros_msgs::srv::EndpointAdd>::SharedPtr add_service;
  rclcpp::Service<mavros_msgs::srv::EndpointDel>::SharedPtr del_service;
  rclcpp::TimerBase::SharedPtr reconnect_timer;
  rclcpp::TimerBase::SharedPtr stale_addrs_timer;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr set_parameters_handle_ptr;

  void add_endpoint(
    const mavros_msgs::srv::EndpointAdd::Request::SharedPtr request,
    mavros_msgs::srv::EndpointAdd::Response::SharedPtr response);
  void del_endpoint(
    const mavros_msgs::srv::EndpointDel::Request::SharedPtr request,
    mavros_msgs::srv::EndpointDel::Response::SharedPtr response);

  void periodic_reconnect_endpoints();
  void periodic_clear_stale_remote_addrs();

  rcl_interfaces::msg::SetParametersResult on_set_parameters_cb(
    const std::vector<rclcpp::Parameter> & parameters);
};

/**
 * MAVConnEndpoint implements Endpoint for FCU or GCS connection
 * via good old libmavconn url's
 *
 * TODO(vooon): support multiple remotes on UDP,
 *              choose right TCP client instead of simple broadcast
 */
class MAVConnEndpoint : public Endpoint
{
public:
  MAVConnEndpoint()
  : Endpoint()
  {
  }

  ~MAVConnEndpoint()
  {
    close();
  }

  mavconn::MAVConnInterface::Ptr link;       // connection

  bool is_open() override;
  std::pair<bool, std::string> open() override;
  void close() override;

  void send_message(const mavlink_message_t * msg, const Framing framing = Framing::ok) override;
};

/**
 * ROSEndpoint implements Endpoint for UAS node
 *
 * That endpoint converts mavlink messages to ROS2 IDL
 * and passes them trough DDL messaging or intra-process comms.
 *
 * Each drone would have separate UAS node
 */
class ROSEndpoint : public Endpoint
{
public:
  ROSEndpoint()
  : Endpoint()
  {
  }

  ~ROSEndpoint()
  {
    close();
  }

  rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr to;       // UAS -> FCU
  rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr from;        // FCU -> UAS

  bool is_open() override;
  std::pair<bool, std::string> open() override;
  void close() override;

  void send_message(const mavlink_message_t * msg, const Framing framing = Framing::ok) override;

private:
  void ros_recv_message(const mavros_msgs::msg::Mavlink::SharedPtr rmsg);
};

} // namespace router
} // namespace mavros

#endif // MAVROS_MAVROS_ROUTER_HPP_

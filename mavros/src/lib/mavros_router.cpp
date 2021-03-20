/**
 * @brief Mavros Router class
 * @file mavros_router.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_router.hpp>
#include <rcpputils/asserts.hpp>

using namespace mavros::router;
using rclcpp::QoS;

static std::atomic<id_t> Router::id_counter {0};

void Router::route_message(
  Endpoint::SharedPtr src, const mavlink_message_t * msg,
  const Framing framing)
{
}

void Router::add_endpoint(
  const mavros_msgs::srv::EndpointAdd::Request::SharedPtr request,
  mavros_msgs::srv::EndpointAdd::Response::SharedPtr response)
{
  auto lg = get_logger();

  RCLCPP_INFO(lg, "Requested to add endpoint: type %d, url: %s", request->type, request->url);

  id_t id = id_counter.fetch_add(1);

  Endpoint::SharedPtr ep;
  if (request->type == mavros_msgs::srv::EndpointAdd::TYPE_UAS) {
    ep = ROSEndpoint::make_shared();
  } else {
    ep = MAVConnEndpoint::make_shared();
  }

  ep.parent = shared_from_this();
  ep.id = id;
  ep.link_type = static_cast<Endpoint::Type>(request->type);
  ep.url = request->url;

  this->endpoints[id] = ep;

  RCLCPP_INFO(lg, "Endpoint link[%d] created", id);

  response.id = id;
  response.success = ep.open();
}

void Router::del_endpoint(
  const mavros_msgs::srv::EndpointDel::Request::SharedPtr request,
  mavros_msgs::srv::EndpointDel::Response::SharedPtr response)
{
  auto lg = get_logger();

  RCLCPP_INFO(lg, "Requested to del endpoint id: %d", request->id);

  auto it = this->endpoints.find(request->id);
  if (it != this->endpoints.end() ) {
    it->second->close();
    this->endpoint.erase(it);
    response->success = true;
  }
}


rcl_interface::msg::SetParametersResult Router::on_set_parameters_cb(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto lg = get_logger();
  rcl_interface::msg::SetParametersResult result;

  RCLCPP_INFO(lg, "params");

  result.success = true;
  for (const auto & parameter : parameters) {
  }

  return result;
}

void Router::periodic_reconnect_endpoints()
{
  auto lg = get_logger();

  RCLCPP_INFO(lg, "reconnecting");
}

void Endpoint::recv_message(const mavlink_message_t * msg, const Framing framing)
{
  rcpputils::assert_true(msg, "msg not nullptr");
  //rcpputils::assert_true(this->parent, "parent not nullptr");

  const addr_t sysid_addr = msg->sysid << 8;
  const addr_t sysid_compid_addr = (msg->sysid << 8) | msg->compid;

  auto sp = this->remote_addrs.emplace(sysid_addr);
  auto scp = this->remote_addrs.emplace(sysid_compid_addr);

  if (auto nh = this->parent.lock()) {
    if (sp.second || scp.second) {
      RCLCPP_INFO(
        nh->get_logger(), "link[%d] detected remote address %d.%d", this->id, msg->sysid,
        msg->compid);
    }

    nh->route_message(shared_from_this(), msg, framing);
  }
}

bool MAVConnEndpoint::is_open()
{
  if (!this->link) {
    return false;
  }

  return this->link->is_open();
}

bool MAVConnEndpoint::open()
{
  auto link = mavconn::MAVConnInterface::open_url(this->url);   // XXX may trow DeviceError
  link->message_received_cb = std::bind(&MAVConnEndpoint::recv_message, shared_from_this(), _1, _2);

  this->link = link;
  return true;
}

void MAVConnEndpoint::close()
{
  if (!this->link) {
    return;
  }

  this->link->close();
  this->link.reset();
}

void MAVConnEndpoint::send_message(const mavlink_message_t * msg, const Framing framing)
{
  if (!this->link) {
    return;
  }

  this->link->send_message_ignore_drop(msg);
}

bool ROSEndpoint::is_open()
{
  return this->from && this->to;
}

bool ROSEndpoint::open()
{
  auto nh = this->parent.lock();
  if (!nh) {
    return false;
  }

  this->from =
    nh->create_publisher<mavros_msgs::msg::Mavlink>(
    utils::format(
      "%s/%s", this->url,
      "mavlink_from"), QoS(
      1000).best_effort());
  this->to = nh->create_subscription<mavros_msgs::msg::Mavlink>(
    utils::format("%s/%s", this->url, "mavlink_to"), QoS(1000).best_effort(),
    std::bind(&ROSEndpoint::ros_recv_message, this, _1));

  return true;
}

void ROSEndpoint::close()
{
  this->from.reset();
  this->to.reset();
}

void ROSEndpoint::send_message(const mavlink_message_t * msg, const Framing framing)
{
  rcpputils::assert_true(msg, "msg not null");

  auto rmsg = mavros_msgs::msg::Mavlink();
  auto success = mavros_msgs::mavlink::convert(*msg, rmsg, utils::enum_value(framing));

  if (success) {
    this->from->publish(rmsg);
  } else if (auto nh = this->parent.lock()) {
    RCLCPP_ERROR(nh->get_logger(), "message conversion error");
  }
}

void ROSEndpoint::ros_recv_message(const mavros_msgs::msg::Mavlink::SharedPtr rmsg)
{
  rcpputils::assert_true(!!rmsg, "rmsg not nullptr");

  mavlink::mavlink_message_t mmsg;

  auto success = mavros_msgs::mavlink::convert(*rmsg, mmsg);
  auto framing = static_cast<Framing>(rmsg->framing_status);

  if (success) {
    recv_message(&mmsg, framing);
  } else if (auto nh = this->parent.lock()) {
    RCLCPP_ERROR(nh->get_logger(), "message conversion error");
  }
}

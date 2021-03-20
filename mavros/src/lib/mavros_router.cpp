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

using unique_lock = std::unique_lock<std::shared_timed_mutex>;
using shared_lock = std::shared_lock<std::shared_timed_mutex>;

std::atomic<id_t> Router::id_counter {1000};

static inline uint8_t get_msg_byte(const mavlink_message_t * msg, uint8_t offset)
{
  uint8_t ret;
  std::memcpy(&ret, static_cast<const void *>(msg->payload64), sizeof(ret));
  return ret;
}

void Router::route_message(
  Endpoint::SharedPtr src, const mavlink_message_t * msg,
  const Framing framing)
{
  shared_lock lock(mu);
  this->stat_msg_routed++;

  // find message destination target
  addr_t target_addr = 0;
  auto msg_entry = ::mavlink::mavlink_get_msg_entry(msg->msgid);
  if (msg_entry) {
    if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM) {
      target_addr |= get_msg_byte(msg, msg_entry->target_system_ofs) << 8;
    }
    if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT) {
      target_addr |= get_msg_byte(msg, msg_entry->target_component_ofs);
    }
  }

  size_t sent_cnt = 0, retry_cnt = 0;
retry:

  for (auto & kv:this->endpoints) {
    auto & dest = kv.second;

    if (src == dest) {
      continue;     // do not echo message
    }
    if (src->link_type == dest->link_type) {
      continue;     // drop messages between same type FCU/GCS/UAS
    }

    bool has_target = dest->remote_addrs.find(target_addr) != dest->remote_addrs.end();

    if (has_target) {
      dest->send_message(msg, framing);
      sent_cnt++;
    }
  }

  // if message haven't been sent retry broadcast it
  if (sent_cnt == 0 && retry_cnt < 2) {
    target_addr = 0;
    retry_cnt++;
    goto retry;
  }

  // update stats
  this->stat_msg_sent.fetch_add(sent_cnt);
  if (sent_cnt == 0) {
    this->stat_msg_dropped++;

    auto lg = get_logger();
    auto clock = get_clock();

    RCLCPP_WARN_THROTTLE(
      lg,
      *clock, 10000, "Message dropped: msgid: %d, source: %d.%d, target: %d.%d", msg->msgid,
      msg->sysid, msg->compid, target_addr >> 8,
      target_addr & 0xff);
  }
}

void Router::add_endpoint(
  const mavros_msgs::srv::EndpointAdd::Request::SharedPtr request,
  mavros_msgs::srv::EndpointAdd::Response::SharedPtr response)
{
  unique_lock lock(mu);
  auto lg = get_logger();

  RCLCPP_INFO(
    lg, "Requested to add endpoint: type: %d, url: %s", request->type,
    request->url.c_str());

  id_t id = id_counter.fetch_add(1);

  RCLCPP_INFO(lg, "link[%d] 1", id);

  Endpoint::SharedPtr ep;
  if (request->type == mavros_msgs::srv::EndpointAdd::Request::TYPE_UAS) {
    ep = std::make_shared<ROSEndpoint>();
  } else {
    ep = std::make_shared<MAVConnEndpoint>();
  }

  RCLCPP_INFO(lg, "link[%d] 2: %p", id, ep);

  // NOTE(vooon): has type std::shared_ptr<rclcpp::Node>
  auto shared_this = shared_from_this();

  ep->parent = std::static_pointer_cast<Router>(shared_this);
  ep->id = id;
  ep->link_type = static_cast<Endpoint::Type>(request->type);
  ep->url = request->url;

  RCLCPP_INFO(lg, "link[%d] 3: %p", id, ep);

  this->endpoints[id] = ep;

  RCLCPP_INFO(lg, "Endpoint link[%d] created", id);

  response->id = id;
  response->success = ep->open();
}

void Router::del_endpoint(
  const mavros_msgs::srv::EndpointDel::Request::SharedPtr request,
  mavros_msgs::srv::EndpointDel::Response::SharedPtr response)
{
  unique_lock lock(mu);
  auto lg = get_logger();

  if (request->id != 0) {
    RCLCPP_INFO(lg, "Requested to del endpoint id: %d", request->id);
    auto it = this->endpoints.find(request->id);
    if (it != this->endpoints.end() ) {
      it->second->close();
      this->endpoints.erase(it);
      response->success = true;
    }
    return;
  }

  RCLCPP_INFO(
    lg, "Requested to del endpoint type: %d url: %s", request->type,
    request->url.c_str());
  for (auto it = this->endpoints.cbegin(); it != this->endpoints.cend(); it++) {
    if (it->second->url == request->url &&
      it->second->link_type == static_cast<Endpoint::Type>( request->type))
    {
      it->second->close();
      this->endpoints.erase(it);
      response->success = true;
      return;
    }
  }
}

rcl_interfaces::msg::SetParametersResult Router::on_set_parameters_cb(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto lg = get_logger();
  rcl_interfaces::msg::SetParametersResult result{};

  RCLCPP_INFO(lg, "params");

  using Type = Endpoint::Type;

  auto get_existing_set = [this](Type type) -> std::set<std::string> {
      shared_lock lock(this->mu);

      std::set<std::string> ret;

      for (const auto & kv : this->endpoints) {
        if (kv.second->link_type != type) {
          continue;
        }

        ret.emplace(kv.second->url);
      }

      return ret;
    };

  auto update_endpoints = [&](const rclcpp::Parameter & parameter, Type type) {
      RCLCPP_DEBUG(lg, "Processing urls parameter: %s", parameter.get_name().c_str());

      auto urls = parameter.as_string_array();
      std::set<std::string> urls_set(urls.begin(), urls.end());
      auto existing_set = get_existing_set(type);

      std::set<std::string> to_add{}, to_del{};
      std::set_difference(
        urls_set.begin(), urls_set.end(), existing_set.begin(),
        existing_set.end(), std::inserter(to_add, to_add.begin()));
      std::set_difference(
        existing_set.begin(), existing_set.end(), urls_set.begin(),
        urls_set.end(), std::inserter(to_del, to_del.begin()));

      for (auto url : to_add) {
        auto req = std::make_shared<mavros_msgs::srv::EndpointAdd::Request>();
        auto resp = std::make_shared<mavros_msgs::srv::EndpointAdd::Response>();

        req->type = utils::enum_value(type);
        req->url = url;

        this->add_endpoint(req, resp);
      }

      for (auto url : to_del) {
        auto req = std::make_shared<mavros_msgs::srv::EndpointDel::Request>();
        auto resp = std::make_shared<mavros_msgs::srv::EndpointDel::Response>();

        req->type = utils::enum_value(type);
        req->url = url;

        this->del_endpoint(req, resp);
      }
    };

  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto name = parameter.get_name();
    if (name == "fcu_urls") {
      update_endpoints(parameter, Type::fcu);
    } else if (name == "gcs_urls") {
      update_endpoints(parameter, Type::gcs);
    } else if (name == "uas_urls") {
      update_endpoints(parameter, Type::uas);
    } else {
      result.successful = false;
      result.reason = "unknown parameter";
    }
  }

  return result;
}

void Router::periodic_reconnect_endpoints()
{
  shared_lock lock(mu);
  auto lg = get_logger();

  RCLCPP_DEBUG(lg, "start reconnecting...");

  for (auto & kv : this->endpoints) {
    auto & p = kv.second;

    if (p->is_open()) {
      continue;
    }

    RCLCPP_INFO(lg, "link[%d] trying to reconnect...", p->id);
    p->open();    // XXX may trow DeviceError
  }
}

void Router::periodic_clear_stale_remote_addrs()
{
  unique_lock lock(mu);
  auto lg = get_logger();

  RCLCPP_DEBUG(lg, "clear stale remotes");
  for (auto & kv : this->endpoints) {
    auto & p = kv.second;

    // Step 1: remove any stale addrs that still there (hadn't been removed by Endpoint::recv_message())
    for (auto addr : p->stale_addrs) {
      if (addr != 0) {
        p->remote_addrs.erase(addr);
        RCLCPP_INFO(
          lg, "link[%d] removed stale remote address %d.%d", p->id, addr >> 8,
          addr & 0xff);
      }
    }

    // Step 2: re-initiate stale_addrs
    p->stale_addrs.clear();
    p->stale_addrs.insert(p->remote_addrs.begin(), p->remote_addrs.end());
  }
}

void Endpoint::recv_message(const mavlink_message_t * msg, const Framing framing)
{
  rcpputils::assert_true(msg, "msg not nullptr");
  //rcpputils::assert_true(this->parent, "parent not nullptr");

  const addr_t sysid_addr = msg->sysid << 8;
  const addr_t sysid_compid_addr = (msg->sysid << 8) | msg->compid;

  // save source addr to remote_addrs
  auto sp = this->remote_addrs.emplace(sysid_addr);
  auto scp = this->remote_addrs.emplace(sysid_compid_addr);

  // and delete it from stale_addrs
  this->stale_addrs.erase(sysid_addr);
  this->stale_addrs.erase(sysid_compid_addr);

  auto & nh = this->parent;
  if (sp.second || scp.second) {
    RCLCPP_INFO(
      nh->get_logger(), "link[%d] detected remote address %d.%d", this->id, msg->sysid,
      msg->compid);
  }

  nh->route_message(shared_from_this(), msg, framing);
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
  (void)framing;

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
  auto & nh = this->parent;
  if (!nh) {
    return false;
  }

  this->from =
    nh->create_publisher<mavros_msgs::msg::Mavlink>(
    utils::format(
      "%s/%s", this->url.c_str(),
      "mavlink_from"), QoS(
      1000).best_effort());
  this->to = nh->create_subscription<mavros_msgs::msg::Mavlink>(
    utils::format("%s/%s", this->url.c_str(), "mavlink_to"), QoS(1000).best_effort(),
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
  } else if (auto & nh = this->parent) {
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
  } else if (auto & nh = this->parent) {
    RCLCPP_ERROR(nh->get_logger(), "message conversion error");
  }
}

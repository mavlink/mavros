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

using namespace mavros::router;
using rclcpp::QoS;

void Router::route_message(Endpoint::SharedPtr src, const mavlink_message_t* msg, const Framing framing)
{
}

void Router::endpoint_add(const mavros_msgs::srv::EndpointAdd::Request::SharedPtr request, mavros_msgs::srv::EndpointAdd::Response::SharedPtr response)
{
}

void Router::endpoint_del(const mavros_msgs::srv::EndpointDel::Request::SharedPtr request, mavros_msgs::srv::EndpointDel::Response::SharedPtr response)
{
}

void Endpoint::recv_message(const mavlink_message_t* msg, const Framing framing)
{
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
    auto link = mavconn::MAVConnInterface::open_url(this->url);
    link->message_received_cb = std::bind(&MAVConnEndpoint::recv_message, this, _1, _2);

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

void MAVConnEndpoint::send_message(const mavlink_message_t* msg, const Framing framing)
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

    this->from = nh->create_publisher<mavros_msgs::msg::Mavlink>(utils::format("%s/%s", this->url, "mavlink_from"), QoS(1000).best_effort());
    this->to = nh->create_subscription<mavros_msgs::msg::Mavlink>(utils::format("%s/%s", this->url, "mavlink_to"), QoS(1000).best_effort(), std::bind(&ROSEndpoint::ros_recv_message, this, _1));

    return true;
}

void ROSEndpoint::close()
{
    this->from.reset();
    this->to.reset();
}

void ROSEndpoint::send_message(const mavlink_message_t* msg, const Framing framing)
{
}

void ROSEndpoint::ros_recv_message(const mavros_msgs::msg::Mavlink::SharedPtr rmsg)
{
}

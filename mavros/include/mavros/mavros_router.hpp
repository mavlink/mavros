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

#include <mavros_msgs/msg/mavlink.hpp>
#include <mavros_msgs/srv/endpoint_add.hpp>
#include <mavros_msgs/srv/endpoint_del.hpp>

namespace mavros {
namespace router {

    using id_t = uint32_t;
    using addr_t = uint32_t;

    using mavconn::Framing;
    using ::mavlink::mavlink_message_t;
    using ::mavlink::msgid_t;

    using namespace std::placeholders;

    class Router;

    class Endpoint : public std::enable_shared_from_this<Endpoint> {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(Endpoint)

        enum class Type {
            fcu = 0,
            gcs = 1,
            uas = 2,
        };

        Endpoint()
            : parent()
            , id(0)
            , link_type(Type::fcu)
            , url {}
            , remote_addrs {}
        {
            const addr_t broadcase_addr = 0;

            // Accept broadcasts by default
            remote_addrs.emplace(broadcase_addr);
        }

        std::weak_ptr<Router> parent;

        uint32_t id; // id of the endpoint
        Type link_type;
        std::string url;               // url to open that endpoint
        std::set<addr_t> remote_addrs; // remotes that we heard there

        virtual bool is_open() = 0;
        virtual bool open() = 0;
        virtual void close() = 0;

        virtual void send_message(const mavlink_message_t* msg, const Framing framing = Framing::ok) = 0;
        virtual void recv_message(const mavlink_message_t* msg, const Framing framing = Framing::ok);
    };

    class Router : public rclcpp::Node {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(Router)

        Router(std::string node_name = "mavros_router")
            : rclcpp::Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
            , endpoints {}
        {
            RCLCPP_DEBUG(this->get_logger(), "Start mavros::router::Router initialization...");

            add_service = this->create_service<mavros_msgs::srv::EndpointAdd>("~/add_endpoint", std::bind(&Router::add_endpoint, this, _1, _2));
            del_service = this->create_service<mavros_msgs::srv::EndpointDel>("~/del_endpoint", std::bind(&Router::del_endpoint, this, _1, _2));
        }

        void route_message(Endpoint::SharedPtr src, const mavlink_message_t* msg, const Framing framing);

    private:
        friend class Endpoint;

        static std::atomic<id_t> id_counter;

        // map stores all routing endpoints
        std::unordered_map<id_t, Endpoint::SharedPtr> endpoints;

        rclcpp::Service<mavros_msgs::srv::EndpointAdd>::SharedPtr add_service;
        rclcpp::Service<mavros_msgs::srv::EndpointDel>::SharedPtr del_service;

        void add_endpoint(const mavros_msgs::srv::EndpointAdd::Request::SharedPtr request, mavros_msgs::srv::EndpointAdd::Response::SharedPtr response);
        void del_endpoint(const mavros_msgs::srv::EndpointDel::Request::SharedPtr request, mavros_msgs::srv::EndpointDel::Response::SharedPtr response);

        void periodic_reconnect_endpoints();
    };

    class MAVConnEndpoint : public Endpoint {
    public:
        MAVConnEndpoint()
            : Endpoint()
        {
        }

        ~MAVConnEndpoint()
        {
            close();
        }

        mavconn::MAVConnInterface::Ptr link; // connection

        bool is_open() override;
        bool open() override;
        void close() override;

        void send_message(const mavlink_message_t* msg, const Framing framing = Framing::ok) override;
    };

    class ROSEndpoint : public Endpoint {
    public:
        ROSEndpoint()
            : Endpoint()
        {
        }

        ~ROSEndpoint()
        {
            close();
        }

        rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr to; // UAS -> FCU
        rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr from;  // FCU -> UAS

        bool is_open() override;
        bool open() override;
        void close() override;

        void send_message(const mavlink_message_t* msg, const Framing framing = Framing::ok) override;

    private:
        void ros_recv_message(const mavros_msgs::msg::Mavlink::SharedPtr rmsg);
    };

} // namespace router
} // namespace mavros

#endif // MAVROS_MAVROS_ROUTER_HPP_

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
#include <mavconn/interface.h>
#include <mavconn/mavlink_dialect.hpp>
#include <mavros/mavlink_diag.h>
#include <mavros/utils.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mavros {
    namespace router {

        using id_t = uint32_t;
        using addr_t = uint32_t;

        using mavconn::Framing;
        using ::mavlink::mavlink_message_t;
        using ::mavlink::msgid_t;

        class Endpoint;

        class Router : public rclcpp::Node {
            public:
                RCLCPP_SMART_PTR_DEFINITIONS(Router)

                    Router(std::string node_name = "mavros_router")
                    : rclcpp::Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
                    , endpoints {}
                , routing {}
                {
                }

                void route_message(Endpoint::Ptr src, bool update_table, const mavlink_message_t* msg, const Framing framing);

            private:
                friend class Endpoint;

                std::unordered_map<id_t, Endpoint::Ptr> endpoints;
                std::unordered_map<addr_t, std::set<id_t>> routing;
        };

        class Endpoint {
            public:
                RCLCPP_SMART_PTR_DEFINITIONS(Endpoint)

                    enum class Type {
                        fcu = 0;
                        gcs = 1;
                        uas = 2;
                    };

                Endpoint()
                    : parent(nullptr)
                      , id(0)
                      , link_type(Type::fcu)
                      , url {}
                , remote_addrs {}
                {
                }

                std::weak_ptr<Router> parent;

                uint32_t id; // id of the endpoint
                Type link_type;
                std::string url;               // url to open that endpoint
                bool is_open;
                std::set<addr_t> remote_addrs; // remotes that we heard there

                virtual bool is_open() = 0;
                virtual bool open() = 0;
                virtual void close() = 0;

                virtual void send_message(const mavlink_message_t* msg, const Framing framing = Framing::ok) = 0;
                virtual void recv_message(const mavlink_message_t* msg, const Framing framing = Framing::ok);
        }

        class MAVConnEndpoint : public Endpoint {
            public:
                MAVConnEndpoint()
                    : Endpoint()
                {
                }

                mavconn::MAVConnInterface::Ptr link; // connection

                override bool is_open();
                override bool open();
                override void close();

                override void send_message(const mavlink_message_t* msg, const Framing framing = Framing::ok);
        };

        class ROSEndpoint : public Endpoint {
            public:
                ROSEndpoint()
                    : Endpoint()
                {
                }

                rclcpp::Subscriber<mavros_msgs::msg::Mavlink>::SharedPtr to;
                rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr from;


                override bool is_open();
                override bool open();
                override void close();


                override void send_message(const mavlink_message_t* msg, const Framing framing = Framing::ok);

                void ros_recv_message(const mavros_msgs::msg::Mavlink::SharedPtr *rmsg);
        }

    }; // namespace router
}; // namespace mavros

#ifndef MAVROS_MAVROS_ROUTER_HPP_

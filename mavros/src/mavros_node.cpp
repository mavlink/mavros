/*
 * Copyright 2013,2014,2015,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS Node
 * @file mavros_node.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#include <string>
#include <memory>
#include <vector>

#include "mavros/mavros_router.hpp"
#include "mavros/mavros_uas.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * MAVROS Node is a transition helper, a component loader preconfigured
 * to work similar to mavros v1.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);

  rclcpp::NodeOptions options;
  // options.use_intra_process_comms(true);

  std::string fcu_url, gcs_url, uas_url;
  int tgt_system = 1, tgt_component = 1;

  auto node = std::make_shared<rclcpp::Node>("mavros_node", options);
  exec.add_node(node);

  node->declare_parameter("fcu_url", fcu_url);
  node->declare_parameter("gcs_url", gcs_url);
  node->declare_parameter("tgt_system", tgt_system);
  node->declare_parameter("tgt_component", tgt_component);

  node->get_parameter("fcu_url", fcu_url);
  node->get_parameter("gcs_url", gcs_url);
  node->get_parameter("tgt_system", tgt_system);
  node->get_parameter("tgt_component", tgt_component);

  uas_url = mavros::utils::format("/uas%d", tgt_system);

  RCLCPP_INFO(node->get_logger(), "Starting mavros_node container");
  RCLCPP_INFO(node->get_logger(), "FCU URL: %s", fcu_url.c_str());
  RCLCPP_INFO(node->get_logger(), "GCS URL: %s", gcs_url.c_str());
  RCLCPP_INFO(node->get_logger(), "UAS Prefix: %s", uas_url.c_str());

  RCLCPP_INFO(node->get_logger(), "Starting mavros router node");
  auto router_node = std::make_shared<mavros::router::Router>(options, "mavros_router");
  exec.add_node(router_node);

  {
    std::vector<rclcpp::Parameter> router_params{};

    if (fcu_url != "") {
      router_params.emplace_back("fcu_urls", std::vector<std::string>{fcu_url});
    }
    if (gcs_url != "") {
      router_params.emplace_back("gcs_urls", std::vector<std::string>{gcs_url});
    }
    router_params.emplace_back("uas_urls", std::vector<std::string>{uas_url});

    router_node->set_parameters(router_params);
  }

  RCLCPP_INFO(node->get_logger(), "Starting mavros uas node");
  auto uas_node = std::make_shared<mavros::uas::UAS>(
    options, "mavros", uas_url, tgt_system,
    tgt_component);
  exec.add_node(uas_node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}

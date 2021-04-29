/*
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS Plugin methods
 * @file plugin.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#include <vector>

#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"

using  mavros::plugin::Plugin;

void Plugin::enable_connection_cb()
{
  uas->add_connection_change_handler(
    std::bind(
      &Plugin::connection_cb, this,
      std::placeholders::_1));
}

void Plugin::enable_capabilities_cb()
{
  uas->add_capabilities_change_handler(
    std::bind(
      &Plugin::capabilities_cb, this,
      std::placeholders::_1));
}

Plugin::SetParametersResult Plugin::node_on_set_parameters_cb(
  const std::vector<rclcpp::Parameter> & parameters)
{
  SetParametersResult result;

  result.successful = true;

  for (auto & p : parameters) {
    auto it = node_watch_parameters.find(p.get_name());
    if (it != node_watch_parameters.end()) {
      try {
        it->second(p);
      } catch (std::exception & ex) {
        result.successful = false;
        result.reason = ex.what();
        break;
      }
    }
  }

  return result;
}

void Plugin::enable_node_watch_parameters()
{
  node_set_parameters_handle_ptr =
    node->add_on_set_parameters_callback(
    std::bind(
      &Plugin::node_on_set_parameters_cb, this, std::placeholders::_1));
}

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

#include <algorithm>
#include <string>
#include <vector>

#include "rcl/arguments.h"
#include "rclcpp/parameter_map.hpp"
#include "rcpputils/scope_exit.hpp"

#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"

using  mavros::plugin::Plugin;

Plugin::Plugin(UASPtr uas_)
: uas(uas_), node(uas_->shared_from_this())
{
}

Plugin::Plugin(
  UASPtr uas_, const std::string & subnode,
  const rclcpp::NodeOptions & options)
: uas(uas_)
{
  // Dynamically created plugin nodes must not inherit process-global
  // __node/__ns remap rules (a component container remaps itself with
  // e.g. -r __node:=<container>), otherwise every plugin node is renamed.
  rclcpp::NodeOptions node_options(options);
  node_options.use_global_arguments(false);
  node_options.use_intra_process_comms(true);

  // Turning off global arguments also drops the process's parameter sources
  // (--params-file / -p). Re-apply the overrides that match this plugin's
  // fully-qualified name as node-local parameter overrides, which carry no
  // remap rules. Mirrors rclcpp's own resolve_parameter_overrides(): global
  // sources first, then the caller-provided overrides take precedence.
  auto context = uas_->get_node_base_interface()->get_context()->get_rcl_context();
  rcl_params_t * global_params = nullptr;
  rcl_ret_t ret = rcl_arguments_get_param_overrides(&context->global_arguments, &global_params);
  if (RCL_RET_OK == ret && nullptr != global_params) {
    auto cleanup = rcpputils::make_scope_exit(
      [global_params]() {rcl_yaml_node_struct_fini(global_params);});

    const std::string fqn =
      std::string(uas_->get_fully_qualified_name()) + "/" + subnode;
    auto param_map = rclcpp::parameter_map_from(global_params, fqn.c_str());

    std::vector<rclcpp::Parameter> merged{};
    auto it = param_map.find(fqn);
    if (it != param_map.end()) {
      merged = it->second;
    }

    for (auto & p : node_options.parameter_overrides()) {
      merged.erase(
        std::remove_if(
          merged.begin(), merged.end(),
          [&p](const rclcpp::Parameter & q) {return q.get_name() == p.get_name();}),
        merged.end());
      merged.push_back(p);
    }

    node_options.parameter_overrides(merged);
  }

  node = rclcpp::Node::make_shared(subnode, uas_->get_fully_qualified_name(), node_options);
}

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

/**
 * @brief MAVROS Plugin methods
 * @file plugin.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>

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

/**
 * @brief MAVROS Plugin register macro
 * @file mavros_plugin_register_macro.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 *  @brief MAVROS Plugin system
 */
/*
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#ifndef MAVROS_MAVROS_PLUGIN_REGISTER_MACRO_HPP_
#define MAVROS_MAVROS_PLUGIN_REGISTER_MACRO_HPP_

#include <class_loader/class_loader.hpp>
#include <mavros/mavros_plugin.hpp>

/**
 * Register a plugin for mavros UAS.
 */
#define MAVROS_PLUGIN_REGISTER(PluginClass) \
  CLASS_LOADER_REGISTER_CLASS( \
    mavros::plugin::PluginFactoryTemplate<PluginClass>, \
    mavros::plugin::PluginFactory)

#endif  // MAVROS_MAVROS_PLUGIN_REGISTER_MACRO_HPP_

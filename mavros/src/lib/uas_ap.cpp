/*
 * Copyright 2014,2015,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS UAS autipilot info methods
 * @file uas_data.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#include <array>
#include <unordered_map>
#include <stdexcept>

#include "mavros/mavros_uas.hpp"
#include "mavros/utils.hpp"
#include "mavros/px4_custom_mode.hpp"

using namespace mavros;         // NOLINT
using namespace mavros::uas;    // NOLINT

/* -*- heartbeat handlers -*- */

void UAS::update_heartbeat(uint8_t type_, uint8_t autopilot_, uint8_t base_mode_)
{
  type = type_;
  autopilot = autopilot_;
  base_mode = base_mode_;
}

void UAS::update_connection_status(bool conn_)
{
  if (conn_ != connected) {
    connected = conn_;

    s_shared_lock lock(mu);

    // call all change cb's
    for (auto & cb : connection_cb_vec) {
      cb(conn_);
    }
  }
}

void UAS::add_connection_change_handler(UAS::ConnectionCb cb)
{
  s_unique_lock lock(mu);
  connection_cb_vec.push_back(cb);
}

/* -*- autopilot version -*- */

static uint64_t get_default_caps([[maybe_unused]] uas::MAV_AUTOPILOT ap_type)
{
  // TODO(vooon): return default caps mask for known FCU's
  return 0;
}

uint64_t UAS::get_capabilities()
{
  if (fcu_caps_known) {
    uint64_t caps = fcu_capabilities;
    return caps;
  } else {
    return get_default_caps(get_autopilot());
  }
}

// This function may need a mutex now
void UAS::update_capabilities(bool known, uint64_t caps)
{
  bool process_cb_queue = false;

  if (known != fcu_caps_known) {
    if (!fcu_caps_known) {
      process_cb_queue = true;
    }
    fcu_caps_known = known;
  } else if (fcu_caps_known) {          // Implies fcu_caps_known == known
    if (caps != fcu_capabilities) {
      process_cb_queue = true;
    }
  } else {}     // Capabilities werent known before and arent known after update

  if (process_cb_queue) {
    fcu_capabilities = caps;

    s_shared_lock lock(mu);
    for (auto & cb : capabilities_cb_vec) {
      cb(static_cast<MAV_CAP>(caps));
    }
  }
}

void UAS::add_capabilities_change_handler(UAS::CapabilitiesCb cb)
{
  s_unique_lock lock(mu);
  capabilities_cb_vec.push_back(cb);
}

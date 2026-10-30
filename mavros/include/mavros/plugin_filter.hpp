/*
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS Plugin message filters
 * @file plugin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 *  @brief MAVROS Plugin system
 */

#pragma once

#ifndef MAVROS__PLUGIN_FILTER_HPP_
#define MAVROS__PLUGIN_FILTER_HPP_

#include <algorithm>

#include <mavconn/interface.hpp>
#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>

namespace mavros
{
namespace plugin
{
namespace filter
{
using mavros::plugin::Filter;
using mavros::uas::UAS;
using UASPtr = UAS *;
using mavconn::Framing;


//! AnyOk filter passes all messages with Framing::ok
class AnyOk : public Filter
{
public:
  inline bool operator()(
    UASPtr uas, const mavlink::mavlink_message_t * cmsg,
    const Framing framing) override
  {
    (void)uas;
    (void)cmsg;
    return framing == Framing::ok;
  }
};

//! SystemAndOk filter passes only messages with Framing::ok and matching target system id
class SystemAndOk : public Filter
{
public:
  inline bool operator()(
    UASPtr uas, const mavlink::mavlink_message_t * cmsg,
    const Framing framing) override
  {
    return framing == Framing::ok && uas->is_my_target(cmsg->sysid);
  }
};

//! ComponentAndOk filter passes only messages with Framing::ok and matching target
//  system/component ids
class ComponentAndOk : public Filter
{
public:
  inline bool operator()(
    UASPtr uas, const mavlink::mavlink_message_t * cmsg,
    const Framing framing) override
  {
    return framing == Framing::ok && uas->is_my_target(cmsg->sysid, cmsg->compid);
  }
};

//! AddressedToUsAndOk filter passes what SystemAndOk passes, if the message's
//  target system/component ids (when it has them) are ours or broadcast (0).
//  It only knows the messages of the common dialect: any other message,
//  e.g. one only in ardupilotmega, is always dropped.
class AddressedToUsAndOk : public SystemAndOk
{
public:
  //! Target ids of a raw message are ours or broadcast (0).
  static bool targets_us(UASPtr uas, const mavlink::mavlink_message_t * cmsg)
  {
    // Look the message up in the common dialect table directly: every message
    // this filter is used for is in it, and mavconn's mavlink_get_msg_entry() only
    // works once a MAVConn link has initialised its table, which a UAS running
    // without a router in the same process never does.
    const auto & entries = mavlink::common::MESSAGE_ENTRIES;
    const auto entry = std::lower_bound(
      entries.begin(), entries.end(), cmsg->msgid,
      [](const mavlink::mavlink_msg_entry_t & e, uint32_t id) {return e.msgid < id;});
    if (entry == entries.end() || entry->msgid != cmsg->msgid) {
      return false;  // unknown here: not a message this filter is meant for
    }

    uint8_t tgt_system = 0, tgt_component = 0;
    if ((entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM) &&
      entry->target_system_ofs < cmsg->len)
    {
      tgt_system = _MAV_PAYLOAD(cmsg)[entry->target_system_ofs];
    }
    if ((entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT) &&
      entry->target_component_ofs < cmsg->len)
    {
      tgt_component = _MAV_PAYLOAD(cmsg)[entry->target_component_ofs];
    }

    return uas->is_addressed_to_us(tgt_system, tgt_component);
  }

  inline bool operator()(
    UASPtr uas, const mavlink::mavlink_message_t * cmsg,
    const Framing framing) override
  {
    return SystemAndOk::operator()(uas, cmsg, framing) && targets_us(uas, cmsg);
  }
};

}   // namespace filter
}   // namespace plugin
}   // namespace mavros


#endif  // MAVROS__PLUGIN_FILTER_HPP_

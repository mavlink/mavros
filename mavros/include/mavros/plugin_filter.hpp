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
using UASPtr = UAS::SharedPtr;
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

}   // namespace filter
}   // namespace plugin
}   // namespace mavros


#endif  // MAVROS__PLUGIN_FILTER_HPP_

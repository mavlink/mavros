//
// Copyright 2015,2016,2021 Vladimir Ermakov.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//
/**
 * @brief Mavlink convert utils
 * @file
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#pragma once
#ifndef MAVROS_MSGS__MAVLINK_CONVERT_HPP_
#define MAVROS_MSGS__MAVLINK_CONVERT_HPP_

#include <mavconn/mavlink_dialect.hpp>
#include <mavros_msgs/msg/mavlink.hpp>

#include <algorithm>

namespace mavros_msgs
{
namespace mavlink
{

using ::mavlink::mavlink_message_t;
using mavros_msgs::msg::Mavlink;

// [[[cog:
// FIELD_NAMES = [
//   "magic",
//   "len",
//   "incompat_flags",
//   "compat_flags",
//   "seq",
//   "sysid",
//   "compid",
//   "msgid",
//   "checksum",
// ]
// ]]]
// [[[end]]] (checksum: d41d8cd98f00b204e9800998ecf8427e)

/**
 * @brief Convert mavros_msgs/Mavlink message to mavlink_message_t
 *
 * @note signature vector should be empty for unsigned OR
 *       MAVLINK_SIGNATURE_BLOCK size for signed messages
 *
 * @param[in]  rmsg	mavros_msgs/Mavlink message
 * @param[out] mmsg	mavlink_message_t struct
 * @return true if success
 */
inline bool convert(const Mavlink & rmsg, mavlink_message_t & mmsg)
{
  if (rmsg.payload64.size() > sizeof(mmsg.payload64) / sizeof(mmsg.payload64[0])) {
    return false;
  }

  if (!rmsg.signature.empty() && rmsg.signature.size() != sizeof(mmsg.signature)) {
    return false;
  }

  // [[[cog:
  // for f in FIELD_NAMES:
  //     cog.outl(f"mmsg.{f} = rmsg.{f};")
  // ]]]
  mmsg.magic = rmsg.magic;
  mmsg.len = rmsg.len;
  mmsg.incompat_flags = rmsg.incompat_flags;
  mmsg.compat_flags = rmsg.compat_flags;
  mmsg.seq = rmsg.seq;
  mmsg.sysid = rmsg.sysid;
  mmsg.compid = rmsg.compid;
  mmsg.msgid = rmsg.msgid;
  mmsg.checksum = rmsg.checksum;
  // [[[end]]] (checksum: 0b66f0fc1cd46db0f18a2429c56a6b8c)
  std::copy(rmsg.payload64.begin(), rmsg.payload64.end(), mmsg.payload64);
  std::copy(rmsg.signature.begin(), rmsg.signature.end(), mmsg.signature);

  return true;
}

/**
 * @brief Convert mavlink_message_t to mavros/Mavlink
 *
 * @param[in]  mmsg	mavlink_message_t struct
 * @param[out] rmsg	mavros_msgs/Mavlink message
 * @param[in]  framing_status  framing parse result (OK, BAD_CRC or BAD_SIGNATURE)
 * @return true, this convertion can't fail
 */
inline bool convert(
  const mavlink_message_t & mmsg, Mavlink & rmsg,
  uint8_t framing_status = Mavlink::FRAMING_OK)
{
  const size_t payload64_len = (mmsg.len + 7) / 8;

  rmsg.framing_status = framing_status;

  // [[[cog:
  // for f in FIELD_NAMES:
  //     cog.outl(f"rmsg.{f} = mmsg.{f};")
  // ]]]
  rmsg.magic = mmsg.magic;
  rmsg.len = mmsg.len;
  rmsg.incompat_flags = mmsg.incompat_flags;
  rmsg.compat_flags = mmsg.compat_flags;
  rmsg.seq = mmsg.seq;
  rmsg.sysid = mmsg.sysid;
  rmsg.compid = mmsg.compid;
  rmsg.msgid = mmsg.msgid;
  rmsg.checksum = mmsg.checksum;
  // [[[end]]] (checksum: 64ef6c1af60c622ed427e005d8ca4f2a)
  rmsg.payload64.assign(
    mmsg.payload64,
    mmsg.payload64 + payload64_len);

  // copy signature block only if message is signed
  if (mmsg.incompat_flags & MAVLINK_IFLAG_SIGNED) {
    rmsg.signature.assign(
      mmsg.signature,
      mmsg.signature + sizeof(mmsg.signature));
  } else {
    rmsg.signature.clear();
  }

  return true;
}

}  // namespace mavlink
}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MAVLINK_CONVERT_HPP_

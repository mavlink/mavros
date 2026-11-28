//
// mavros
// Copyright 2026 Vishwa (vishwagw), All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Helpers shared by the mavros_extras plugin tests, on top of the core
 * `TestUAS` fixture from mavros/test/test_plugin_helpers.hpp.
 */

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "test_plugin_helpers.hpp"

namespace mavros
{
namespace uas
{

//! Messages with the given msgid in a sink capture, decoded as MAVLink type M.
template<class M>
std::vector<M> sent_messages(const TestUAS::CaptureBuffer & buf)
{
  std::vector<M> ret;
  for (const auto & rmsg : buf.snapshot()) {
    if (rmsg.msgid != M::MSG_ID) {
      continue;
    }
    auto mmsg = TestUAS::to_mavlink(rmsg);
    mavlink::MsgMap map(mmsg);
    M m{};
    m.deserialize(map);
    ret.push_back(m);
  }
  return ret;
}

//! Wait until the sink capture holds at least @a count messages of type M.
template<class M>
std::vector<M> wait_sent(
  const TestUAS::CaptureBuffer & buf, const size_t count,
  const std::chrono::milliseconds timeout = 5s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto ret = sent_messages<M>(buf);
  while (ret.size() < count && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
    ret = sent_messages<M>(buf);
  }
  return ret;
}

}   // namespace uas
}   // namespace mavros

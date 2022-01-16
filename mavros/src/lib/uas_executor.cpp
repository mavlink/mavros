/*
 * Copyright 2022 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS UAS Node Executor
 * @file uas_executor.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#include <algorithm>

#include "mavros/uas_executor.hpp"

using namespace mavros;                 // NOLINT
using namespace mavros::uas;            // NOLINT
using namespace std::chrono_literals;   // NOLINT

UASExecutor::UASExecutor(const rclcpp::ExecutorOptions & options)
: MultiThreadedExecutor(options, select_number_of_threads(), true, 1000ms),
  source_system(0),
  source_component(0)
{
}

size_t UASExecutor::select_number_of_threads()
{
  // return std::max<size_t>(16, std::min<size_t>(std::thread::hardware_concurrency(), 4));
  return std::clamp<size_t>(std::thread::hardware_concurrency(), 4, 16);
}

void UASExecutor::set_ids(uint8_t sysid, uint8_t compid)
{
  source_system = sysid;
  source_component = compid;
}

void UASExecutor::run(size_t thread_id)
{
  utils::set_this_thread_name("uas-exec/%d.%d/%zu", source_system, source_component, thread_id);
  MultiThreadedExecutor::run(thread_id);
}

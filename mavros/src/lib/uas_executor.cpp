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
#include <charconv>
#include <cstdlib>
#include <string_view>
#include <system_error>

#include "mavros/uas_executor.hpp"

using namespace mavros;                 // NOLINT
using namespace mavros::uas;            // NOLINT
using namespace std::chrono_literals;   // NOLINT

namespace
{
constexpr size_t kMinExecutorThreads = 2;
constexpr size_t kMaxExecutorThreads = 16;
constexpr std::string_view kExecutorThreadsEnv = "MAVROS_UAS_EXECUTOR_THREADS";

size_t configured_number_of_threads()
{
  auto value = std::getenv(kExecutorThreadsEnv.data());
  if (value == nullptr || value[0] == '\0') {
    return 0;
  }

  size_t threads = 0;
  const auto input = std::string_view(value);
  const auto result = std::from_chars(input.data(), input.data() + input.size(), threads);
  if (result.ec != std::errc{} || result.ptr != input.data() + input.size() || threads == 0) {
    return 0;
  }

  return std::clamp(threads, kMinExecutorThreads, kMaxExecutorThreads);
}
}  // namespace

UASExecutor::UASExecutor(const rclcpp::ExecutorOptions & options)
: MultiThreadedExecutor(options, select_number_of_threads(), true, 1000ms),
  source_system(0),
  source_component(0)
{
}

size_t UASExecutor::select_number_of_threads()
{
  auto configured_threads = configured_number_of_threads();
  if (configured_threads > 0) {
    return configured_threads;
  }

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

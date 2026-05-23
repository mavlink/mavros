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
#include <cstdlib>

#include "mavros/uas_executor.hpp"

using namespace mavros;               // NOLINT
using namespace mavros::uas;          // NOLINT
using namespace std::chrono_literals; // NOLINT

UASExecutor::UASExecutor(const rclcpp::ExecutorOptions & options)
: MultiThreadedExecutor(options, select_number_of_threads(), true, 1000ms),
  source_system(0),
  source_component(0)
{
}

size_t UASExecutor::select_number_of_threads()
{
  if (const char *env = std::getenv("MAVROS_UAS_EXECUTOR_THREADS")) {
    try {
      size_t n = std::stoul(env);
      if (n >= 2) {
        RCLCPP_INFO(
            rclcpp::get_logger("uas_executor"),
            "UAS executor threads overridden by MAVROS_UAS_EXECUTOR_THREADS: %zu", n);
        return n;
      }
      RCLCPP_WARN(
          rclcpp::get_logger("uas_executor"),
          "MAVROS_UAS_EXECUTOR_THREADS must be >= 2, got %zu. Using default.", n);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
          rclcpp::get_logger("uas_executor"),
          "Invalid MAVROS_UAS_EXECUTOR_THREADS value '%s': %s. Using default.", env, e.what());
    }
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

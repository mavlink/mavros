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
#include <cstring>
#include <thread>

#include "mavros/uas_executor.hpp"

#ifdef MAVROS_HAVE_EVENTS_CBG_EXECUTOR
#include "rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp"
#endif

using namespace mavros;               // NOLINT
using namespace mavros::uas;          // NOLINT
using namespace std::chrono_literals; // NOLINT

UASExecutor::UASExecutor(const rclcpp::ExecutorOptions & options)
{
  number_of_threads_ = select_number_of_threads();
  impl_ = make_executor(options, number_of_threads_, true, 1000ms);
}

void UASExecutor::spin()
{
  impl_->spin();
}

void UASExecutor::cancel()
{
  impl_->cancel();
}

void UASExecutor::add_node(const std::shared_ptr<rclcpp::Node> & node)
{
  impl_->add_node(node);
}

size_t UASExecutor::get_number_of_threads() const
{
  return number_of_threads_;
}

size_t UASExecutor::select_number_of_threads()
{
  if (const char * env = std::getenv("MAVROS_UAS_EXECUTOR_THREADS")) {
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
  return std::clamp<size_t>(std::thread::hardware_concurrency(), 2, 4);
}

std::unique_ptr<rclcpp::Executor> mavros::uas::make_executor(
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  bool yield_before_execute,
  std::chrono::nanoseconds timeout)
{
  bool use_events = false;
  if (const char * env = std::getenv("MAVROS_EXECUTOR_TYPE")) {
    if (std::strcmp(env, "events") == 0 || std::strcmp(env, "cbg") == 0) {
      use_events = true;
    } else if (std::strcmp(env, "mt") != 0 && std::strcmp(env, "multithreaded") != 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("uas_executor"),
        "Invalid MAVROS_EXECUTOR_TYPE value '%s'. Using MultiThreadedExecutor.", env);
    }
  }

#ifdef MAVROS_HAVE_EVENTS_CBG_EXECUTOR
  if (use_events) {
    RCLCPP_INFO(
      rclcpp::get_logger("uas_executor"),
      "Using EventsCBGExecutor, threads: %zu", number_of_threads);
    return std::make_unique<rclcpp::executors::EventsCBGExecutor>(
      options, number_of_threads, timeout);
  }
#else
  if (use_events) {
    RCLCPP_WARN(
      rclcpp::get_logger("uas_executor"),
      "EventsCBGExecutor requested but not available in this ROS distro. "
      "Using MultiThreadedExecutor.");
  }
#endif

  return std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
    options, number_of_threads, yield_before_execute, timeout);
}

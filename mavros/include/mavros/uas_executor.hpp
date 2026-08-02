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
 *
 * @addtogroup nodelib
 * @{
 */

#pragma once

#ifndef   MAVROS__UAS_EXECUTOR_HPP_
#define   MAVROS__UAS_EXECUTOR_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace mavros
{
namespace uas
{

/**
 * Executor for UAS Plugin nodes
 *
 * Thin wrapper around rclcpp::Executor that lets the user pick the
 * underlying executor type at runtime (MAVROS_EXECUTOR_TYPE env var).
 */
class UASExecutor
{
public:
  explicit UASExecutor(const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());
  ~UASExecutor() = default;

  void spin();
  void cancel();
  void add_node(const std::shared_ptr<rclcpp::Node> & node);
  size_t get_number_of_threads() const;

private:
  RCLCPP_DISABLE_COPY(UASExecutor)

  static size_t select_number_of_threads();

  std::unique_ptr<rclcpp::Executor> impl_;
  size_t number_of_threads_;
};

/**
 * Factory for executors honoring the MAVROS_EXECUTOR_TYPE env var:
 *  - "events" (or "cbg"): rclcpp::executors::EventsCBGExecutor (Lyrical+ only)
 *  - otherwise (default): rclcpp::executors::MultiThreadedExecutor
 *
 * \param options common options for all executors
 * \param number_of_threads number of threads to have in the thread pool
 * \param yield_before_execute MultiThreadedExecutor option, ignored for
 *   the events executor
 * \param timeout maximum time to wait
 */
std::unique_ptr<rclcpp::Executor> make_executor(
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  bool yield_before_execute = false,
  std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

}   // namespace uas
}   // namespace mavros

#endif  // MAVROS__UAS_EXECUTOR_HPP_

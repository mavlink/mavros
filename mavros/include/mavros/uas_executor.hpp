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

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "mavros/utils.hpp"

namespace mavros
{
namespace uas
{

/**
 * Executor for UAS Plugin nodes
 */
class UASExecutor : public rclcpp::executors::MultiThreadedExecutor
{
public:
  explicit UASExecutor(const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());
  ~UASExecutor() = default;

  void set_ids(uint8_t sysid, uint8_t compid);

protected:
  void run(size_t thread_id);

private:
  RCLCPP_DISABLE_COPY(UASExecutor)

  static size_t select_number_of_threads();

  uint8_t source_system, source_component;
};

}   // namespace uas
}   // namespace mavros

#endif  // MAVROS__UAS_EXECUTOR_HPP_

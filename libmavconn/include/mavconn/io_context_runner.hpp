//
// libmavconn
// Copyright 2026 Vladimir Ermakov, contributors.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

#pragma once
#ifndef MAVCONN__IO_CONTEXT_RUNNER_HPP_
#define MAVCONN__IO_CONTEXT_RUNNER_HPP_

#include <atomic>
#include <memory>
#include <thread>
#include <utility>

#include <asio.hpp>

namespace mavconn
{

/**
 * @brief Small utility to unify owned/shared io_service lifecycle handling.
 */
class IoContextRunner
{
public:
  explicit IoContextRunner(asio::io_service * shared_io = nullptr)
  : io_owner_(shared_io ? nullptr : std::make_shared<asio::io_service>()),
    io_(shared_io ? *shared_io : *io_owner_),
    io_work_(shared_io ? nullptr : std::make_unique<asio::io_service::work>(io_)),
    owns_thread_(shared_io == nullptr),
    is_running_(false)
  {}

  asio::io_service & io()
  {
    return io_;
  }

  bool owns_thread() const
  {
    return owns_thread_;
  }

  bool is_running() const
  {
    return is_running_.load();
  }

  template<typename Fn>
  void start(Fn && fn)
  {
    if (!owns_thread_) {
      return;
    }

    io_thread_ = std::thread(
      [this, f = std::forward<Fn>(fn)]() mutable {
        is_running_ = true;
        f();
        is_running_ = false;
      });
  }

  void stop_owned()
  {
    if (!owns_thread_) {
      return;
    }

    io_work_.reset();
    io_.stop();
  }

  void join_owned()
  {
    if (!owns_thread_) {
      return;
    }

    if (!io_thread_.joinable()) {
      return;
    }

    if (std::this_thread::get_id() == io_thread_.get_id()) {
      // Cannot join from the same thread; detach so destructor can't terminate.
      io_thread_.detach();
      return;
    }

    io_thread_.join();
  }

  void reset_owned()
  {
    if (!owns_thread_) {
      return;
    }

    io_.reset();
  }

  void shutdown_owned()
  {
    stop_owned();
    join_owned();
    reset_owned();
  }

private:
  std::shared_ptr<asio::io_service> io_owner_;
  asio::io_service & io_;
  std::unique_ptr<asio::io_service::work> io_work_;
  bool owns_thread_;
  std::atomic<bool> is_running_;
  std::thread io_thread_;
};

}  // namespace mavconn

#endif  // MAVCONN__IO_CONTEXT_RUNNER_HPP_

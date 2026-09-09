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
 * @brief Small utility to unify owned/shared io_context lifecycle handling.
 *
 * When the runner owns its io_context and I/O thread, the worker thread keeps a
 * reference to the shared state (io_context, work guard, is_running). This lets
 * a shutdown initiated from the worker thread itself detach safely: the state
 * survives even if the owning connection is destroyed before the thread exits,
 * and the io_context is restarted only after run() has returned.
 */
class IoContextRunner
{
public:
  explicit IoContextRunner(asio::io_context * shared_io = nullptr)
  : owns_thread_(shared_io == nullptr),
    state_(std::make_shared<State>(shared_io))
  {}

  [[nodiscard]] asio::io_context & io()
  {
    return state_->io();
  }

  [[nodiscard]] bool owns_thread() const
  {
    return owns_thread_;
  }

  [[nodiscard]] bool is_running() const
  {
    return state_->is_running.load();
  }

  template<typename Fn>
  void start(Fn && fn)
  {
    if (!owns_thread_) {
      return;
    }

    // The worker captures the shared state so the io_context and is_running
    // flag outlive the connection when shutdown is initiated from this thread
    // (self-close) and join_owned() has to detach.
    auto state = state_;
    io_thread_ = std::jthread(
      [state, f = std::forward<Fn>(fn)]() mutable {
        state->is_running = true;
        f();
        // io_context::run() has returned, so it is now safe to restart the
        // context. This handles self-initiated shutdown: shutdown_owned()
        // cannot restart while run() is still active on this thread, so the
        // restart is deferred until here.
        state->restart();
        state->is_running = false;
      });
  }

  void stop_owned()
  {
    if (!owns_thread_) {
      return;
    }

    io_thread_.request_stop();
    state_->release_work_guard();
    state_->stop();
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
      // The worker keeps the shared state alive until it completes.
      io_thread_.detach();
      return;
    }

    io_thread_.join();
  }

  void shutdown_owned()
  {
    stop_owned();
    join_owned();
    // The io_context is restarted by the worker thread once run() has
    // returned (see start()). On the self-close path join_owned() detaches,
    // and restarting here while run() is still active on this thread would
    // be undefined behaviour.
  }

private:
  class State
  {
public:
    explicit State(asio::io_context * shared_io)
    : io_owner_(shared_io ? nullptr : std::make_shared<asio::io_context>()),
      io_ref_(shared_io ? *shared_io : *io_owner_),
      io_work_(shared_io ? nullptr :
        std::make_unique<asio::executor_work_guard<asio::io_context::executor_type>>(
        asio::make_work_guard(io_ref_)))
    {}

    [[nodiscard]] asio::io_context & io()
    {
      return io_ref_;
    }

    void release_work_guard()
    {
      io_work_.reset();
    }

    void stop()
    {
      io_ref_.stop();
    }

    void restart()
    {
      io_ref_.restart();
    }

    std::shared_ptr<asio::io_context> io_owner_;
    asio::io_context & io_ref_;
    std::unique_ptr<asio::executor_work_guard<asio::io_context::executor_type>> io_work_;
    std::atomic<bool> is_running{false};
  };

  bool owns_thread_;
  std::shared_ptr<State> state_;
  std::jthread io_thread_;
};

}  // namespace mavconn

#endif  // MAVCONN__IO_CONTEXT_RUNNER_HPP_

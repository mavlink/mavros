/*
 * Copyright 2026 Sean Hickey.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Isolated two-file PX4 event metadata acquisition
 * @file acquisition.hpp
 * @author Sean Hickey
 */

#ifndef PLUGINS__PX4_EVENTS__ACQUISITION_HPP_
#define PLUGINS__PX4_EVENTS__ACQUISITION_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "decoder.hpp"
#include "metadata.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/file_open.hpp"
#include "mavros_msgs/srv/file_read.hpp"
#include "mavros_msgs/srv/file_close.hpp"

namespace mavros
{
namespace px4_events
{

class MetadataCache;

// The caller owns PX4/mode gating, the startup delay, and connection lifecycle.
class Acquisition : public std::enable_shared_from_this<Acquisition>
{
public:
  struct Result
  {
    std::unique_ptr<Decoder> decoder;
    std::string error;
    bool transport_uncertain{false};
  };
  using Callback = std::function<void (Result)>;

  // An omitted cache preserves download-only behavior. Cache access performs
  // bounded synchronous local I/O in its callback group; no worker is added.
  static std::shared_ptr<Acquisition> create(
    const rclcpp::Node::SharedPtr & node, Decoder::ErrorCallback report_error = {},
    std::shared_ptr<const MetadataCache> cache = {});
  ~Acquisition();
  // Schedules cache lookup. False leaves existing work untouched. With the
  // download gate closed, misses wait without starting FTP service deadlines.
  bool start(const MetadataFile & general, Callback callback, bool downloads_allowed = true);
  // Release the owner's connection-based startup gate; performs no I/O inline.
  void allow_downloads();
  // Ignores late service replies and sends no cleanup/recovery requests.
  void cancel();

private:
  friend class AcquisitionTestPeer;
  using Clock = std::chrono::steady_clock;
  using Open = mavros_msgs::srv::FileOpen;
  using Read = mavros_msgs::srv::FileRead;
  using Close = mavros_msgs::srv::FileClose;
  enum class Phase { Idle, Cache, DownloadWait, Services, Open, Read, Close, Retry };
  enum class File { General, Events };
  struct Completion { Callback callback; Result result; };
  static constexpr auto WAIT_TIMEOUT = std::chrono::seconds(10);
  static constexpr auto BUSY_BACKOFF = std::chrono::seconds(2);
  static constexpr size_t CHUNK_SIZE = 239;

  Acquisition(
    const rclcpp::Node::SharedPtr & node, Decoder::ErrorCallback report_error,
    std::shared_ptr<const MetadataCache> cache);
  void initialize();
  void tick(Clock::time_point now);
  // State helpers require mutex_; the sole owner callback runs outside it.
  void begin_file_locked(const MetadataFile & file);
  void begin_download_locked();
  void dispatch_locked();
  template<class Service>
  void send_locked(
    const std::shared_ptr<rclcpp::Client<Service>> & client,
    const std::shared_ptr<typename Service::Request> & request,
    void (Acquisition::* handler)(const typename Service::Response &));
  bool failed_locked(bool success, int error);
  void open_reply_locked(const Open::Response & response);
  void read_reply_locked(const Read::Response & response);
  void close_reply_locked(const Close::Response & response);
  void stop_locked(const std::string & reason, bool uncertain);
  void file_finished_locked(const std::string & close_error = "", bool uncertain = false);
  void save_cache_locked() const;
  void clear_pending_locked();
  void fail_locked(
    const std::string & reason, bool retryable, bool uncertain,
    Clock::time_point now);
  void finish_locked(Result result);
  void deliver();

  rclcpp::Node::SharedPtr node_;
  Decoder::ErrorCallback report_error_;
  std::shared_ptr<const MetadataCache> cache_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Client<Open>::SharedPtr open_;
  rclcpp::Client<Read>::SharedPtr read_;
  rclcpp::Client<Close>::SharedPtr close_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  Phase phase_{Phase::Idle};
  File file_{File::General};
  uint64_t serial_{0};
  std::optional<int64_t> pending_;
  Clock::time_point deadline_{};
  bool busy_retried_{false};
  bool opened_{false};
  bool retryable_{false};
  bool from_cache_{false};
  bool downloads_allowed_{true};
  size_t limit_{0};
  size_t size_{0};
  size_t requested_{0};
  std::string bytes_;
  std::string error_;
  unsigned attempts_{0};
  Clock::time_point next_request_{};
  MetadataFile general_;
  MetadataFile current_;
  Callback callback_;
  std::optional<Completion> completion_;
};

}  // namespace px4_events
}  // namespace mavros

#endif  // PLUGINS__PX4_EVENTS__ACQUISITION_HPP_

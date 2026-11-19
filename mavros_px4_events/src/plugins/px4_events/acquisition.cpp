/*
 * Copyright 2026 Sean Hickey.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Bounded general/event metadata retrieval and validation
 * @file acquisition.cpp
 * @author Sean Hickey
 */

#include "acquisition.hpp"

#include <algorithm>
#include <cerrno>
#include <exception>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "mavros/utils.hpp"
#include "cache.hpp"

namespace mavros
{
namespace px4_events
{

std::shared_ptr<Acquisition> Acquisition::create(
  const rclcpp::Node::SharedPtr & node, Decoder::ErrorCallback report_error,
  std::shared_ptr<const MetadataCache> cache)
{
  auto acquisition = std::shared_ptr<Acquisition>(
    new Acquisition(node, std::move(report_error), std::move(cache)));
  acquisition->initialize();
  return acquisition;
}

Acquisition::Acquisition(
  const rclcpp::Node::SharedPtr & node, Decoder::ErrorCallback report_error,
  std::shared_ptr<const MetadataCache> cache)
: node_(node), report_error_(std::move(report_error)), cache_(std::move(cache))
{}

void Acquisition::initialize()
{
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
#ifdef USE_OLD_RMW_QOS
  auto qos = rmw_qos_profile_services_default;
#else
  auto qos = rclcpp::ServicesQoS();
#endif
  open_ = node_->create_client<Open>("ftp/open", qos, group_);
  read_ = node_->create_client<Read>("ftp/read", qos, group_);
  close_ = node_->create_client<Close>("ftp/close", qos, group_);
  const std::weak_ptr<Acquisition> weak = shared_from_this();
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(250), [weak]() {
        if (const auto self = weak.lock()) {
          self->tick(Clock::now());
        }
    }, group_);
  timer_->cancel();
}

Acquisition::~Acquisition()
{
  if (timer_) {
    timer_->cancel();
  }
  clear_pending_locked();
}

void Acquisition::clear_pending_locked()
{
  if (pending_) {
    switch (phase_) {
      case Phase::Open: open_->remove_pending_request(*pending_); break;
      case Phase::Read: read_->remove_pending_request(*pending_); break;
      case Phase::Close: close_->remove_pending_request(*pending_); break;
      default: break;
    }
    pending_.reset();
  }
}

bool Acquisition::start(const MetadataFile & general, Callback callback, bool downloads_allowed)
{
  bool setup_failed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != Phase::Idle || completion_ || !callback) {
      return false;
    }
    attempts_ = 1;
    general_ = general;
    callback_ = std::move(callback);
    file_ = File::General;
    downloads_allowed_ = downloads_allowed;
    try {
      begin_file_locked(general_);
      if (phase_ != Phase::Idle) {
        timer_->reset();
      }
    } catch (const std::exception & ex) {
      finish_locked({nullptr, ex.what(), false});
      setup_failed = true;
    } catch (...) {
      finish_locked({nullptr, "unknown metadata acquisition setup exception", false});
      setup_failed = true;
    }
  }
  // Cache access belongs to the acquisition callback group, not the caller's
  // advertisement handler. The inline tick only performs immediate FTP dispatch.
  if (!cache_) {
    tick(Clock::now(), false);
  } else if (setup_failed) {
    deliver();
  }
  return true;
}

void Acquisition::allow_downloads()
{
  std::lock_guard<std::mutex> lock(mutex_);
  downloads_allowed_ = true;
  if (phase_ == Phase::DownloadWait) {
    begin_download_locked();
  }
}

void Acquisition::begin_file_locked(const MetadataFile & file)
{
  opened_ = false;
  stopping_ = false;
  stop_uncertain_ = false;
  busy_retried_ = false;
  session_retried_ = false;
  retryable_ = false;
  validation_uncertain_ = false;
  bytes_.clear();
  error_.clear();
  current_ = metadata_file(file.uri, file.crc);
  if (current_.ftp_path.size() > 238) {
    throw std::runtime_error("metadata FTP path exceeds supported length");
  }
  const auto & path = current_.ftp_path;
  limit_ = path.size() >= 3 && path.substr(path.size() - 3) == ".xz" ?
    Decoder::MAX_COMPRESSED_BYTES : Decoder::MAX_JSON_BYTES;
  if (cache_) {
    phase_ = Phase::Cache;
  } else {
    begin_download_locked();
  }
}

void Acquisition::begin_download_locked()
{
  phase_ = downloads_allowed_ ? Phase::Services : Phase::DownloadWait;
  if (downloads_allowed_) {
    deadline_ = Clock::now() + WAIT_TIMEOUT;
  }
}

void Acquisition::cancel()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != Phase::Idle) {
      stopping_ = true;
      stop_uncertain_ = stop_uncertain_ || opened_ || validation_uncertain_ || pending_.has_value();
      retryable_ = false;
      error_ = "metadata acquisition cancelled";
      // Retain an open response to clean up a late success. An existing close
      // keeps its original deadline; repeated cancellation must not extend it.
      if (!(phase_ == Phase::Open && pending_) && phase_ != Phase::Close) {
        stop_locked(error_, stop_uncertain_);
      }
    }
  }
  deliver();
}

void Acquisition::tick(Clock::time_point now, bool process_files)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if (phase_ == Phase::Retry && now >= next_request_) {
        file_ = File::General;
        begin_file_locked(general_);
      }
    } catch (const std::exception & ex) {
      stop_locked(ex.what(), true);
    } catch (...) {
      stop_locked("unknown metadata FTP exception", true);
    }
  }
  if (process_files) {
    process_work();
    now = Clock::now();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if (phase_ == Phase::Services) {
        if (open_->service_is_ready() && read_->service_is_ready() && close_->service_is_ready()) {
          phase_ = Phase::Open;
          dispatch_locked();
        } else if (now >= deadline_) {
          stop_locked("metadata FTP services unavailable", false);
        }
      } else if (pending_) {
        if (now >= deadline_) {
          stop_locked("metadata FTP service response timeout", true);
        }
      } else if (phase_ == Phase::Open || phase_ == Phase::Read || phase_ == Phase::Close) {
        if (now >= next_request_) {
          dispatch_locked();
        }
      }
    } catch (const std::exception & ex) {
      stop_locked(ex.what(), true);
    } catch (...) {
      stop_locked("unknown metadata FTP exception", true);
    }
  }
  // A close timeout can make a completed file ready for validation on this tick.
  if (process_files) {
    process_work();
  }
  deliver();
}

template<class Service>
void Acquisition::send_locked(
  const std::shared_ptr<rclcpp::Client<Service>> & client,
  const std::shared_ptr<typename Service::Request> & request,
  void (Acquisition::* handler)(const typename Service::Response &))
{
  const auto token = ++serial_;
  deadline_ = Clock::now() + WAIT_TIMEOUT;
  const std::weak_ptr<Acquisition> weak = shared_from_this();
  const auto future = client->async_send_request(
    request, [weak, token, handler](typename rclcpp::Client<Service>::SharedFuture response) {
      if (const auto self = weak.lock()) {
        {
          std::lock_guard<std::mutex> lock(self->mutex_);
          if (!self->pending_ || token != self->serial_) {
            return;
          }
          self->clear_pending_locked();
          try {
            (self.get()->*handler)(*response.get());
          } catch (const std::exception & ex) {
            self->stop_locked(ex.what(), true);
          } catch (...) {
            self->stop_locked("unknown metadata FTP response exception", true);
          }
        }
        self->process_work();
        self->deliver();
      }
    });
  pending_ = future.request_id;
}

void Acquisition::dispatch_locked()
{
  if (phase_ == Phase::Open) {
    auto request = std::make_shared<Open::Request>();
    request->file_path = current_.ftp_path;
    request->mode = Open::Request::MODE_READ;
    send_locked<Open>(open_, request, &Acquisition::open_reply_locked);
  } else if (phase_ == Phase::Read) {
    auto request = std::make_shared<Read::Request>();
    request->file_path = current_.ftp_path;
    request->offset = bytes_.size();
    requested_ = std::min(CHUNK_SIZE, size_ - bytes_.size());
    request->size = requested_;
    send_locked<Read>(read_, request, &Acquisition::read_reply_locked);
  } else if (phase_ == Phase::Close) {
    auto request = std::make_shared<Close::Request>();
    request->file_path = current_.ftp_path;
    send_locked<Close>(close_, request, &Acquisition::close_reply_locked);
  }
}

bool Acquisition::failed_locked(bool success, int error)
{
  if (success) {
    busy_retried_ = false;
    return false;
  }
  if (error == EBUSY && !busy_retried_) {
    busy_retried_ = true;
    next_request_ = Clock::now() + BUSY_BACKOFF;
  } else {
    if (phase_ != Phase::Close) {
      retryable_ = error == EBUSY || error == EIO;
    }
    stop_locked(
      "metadata FTP operation failed, errno=" + std::to_string(error),
      error == ETIMEDOUT || error == EILSEQ || error == EBADE ||
      error == EBADSLT || error == EBADRQC);
  }
  return true;
}

void Acquisition::open_reply_locked(const Open::Response & response)
{
  if (stopping_) {
    opened_ = response.success;
    stop_locked(error_, stop_uncertain_);
    return;
  }
  if (!response.success && response.r_errno == EMFILE && !session_retried_) {
    session_retried_ = true;
    next_request_ = Clock::now() + SESSION_BACKOFF;
    RCLCPP_WARN(node_->get_logger(),
      "PX4 metadata FTP has no free session; retrying open once in 10 seconds");
    return;
  }
  // Older PX4 joins its FTP root and a relative path without a separator.
  // Retry once with a rooted path, retaining the advertised URI for the cache.
  // A rooted path also records that this fallback has already been attempted.
  if (!response.success && response.r_errno == ENOENT &&
    current_.ftp_path.front() != '/' && current_.ftp_path.front() != '@')
  {
    if (current_.ftp_path.size() >= 238) {
      stop_locked("metadata FTP rooted fallback path exceeds supported length", false);
      return;
    }
    current_.ftp_path.insert(0, "/");
    busy_retried_ = false;
    RCLCPP_DEBUG(node_->get_logger(), "PX4 metadata FTP retry with rooted path: %s",
      current_.ftp_path.c_str());
    dispatch_locked();
    return;
  }
  if (failed_locked(response.success, response.r_errno)) {
    return;
  }
  opened_ = true;
  size_ = response.size;
  if (size_ == 0 || size_ > limit_) {
    stop_locked("metadata FTP file is empty or exceeds size limit", false);
    return;
  }
  bytes_.reserve(size_);
  phase_ = Phase::Read;
  dispatch_locked();
}

void Acquisition::read_reply_locked(const Read::Response & response)
{
  if (failed_locked(response.success, response.r_errno)) {
    return;
  }
  if (response.data.empty() || response.data.size() > requested_ ||
    response.data.size() > size_ - bytes_.size())
  {
    stop_locked("metadata FTP read made no progress or returned excess data", false);
    return;
  }
  bytes_.append(reinterpret_cast<const char *>(response.data.data()), response.data.size());
  if (bytes_.size() == size_) {
    phase_ = Phase::Close;
  }
  dispatch_locked();  // No pacing: next read starts directly from this response callback.
}

void Acquisition::close_reply_locked(const Close::Response & response)
{
  if (!failed_locked(response.success, response.r_errno)) {
    opened_ = false;
    file_finished_locked({}, stop_uncertain_);
  }
}

void Acquisition::stop_locked(const std::string & reason, bool uncertain)
{
  if (phase_ == Phase::Idle) {
    return;
  }
  if (phase_ == Phase::Close) {
    file_finished_locked(reason, uncertain || stop_uncertain_);
    return;
  }
  stopping_ = true;
  stop_uncertain_ = stop_uncertain_ || uncertain;
  error_ = reason;
  clear_pending_locked();
  if (opened_) {
    busy_retried_ = false;
    phase_ = Phase::Close;
    dispatch_locked();
  } else {
    file_finished_locked({}, stop_uncertain_);
  }
}

void Acquisition::file_finished_locked(const std::string & close_error, bool uncertain)
{
  clear_pending_locked();
  if (!close_error.empty()) {
    RCLCPP_WARN(node_->get_logger(), "PX4 metadata FTP cleanup incomplete: %s",
          close_error.c_str());
  }
  const auto now = Clock::now();
  if (!error_.empty()) {
    const auto error = error_;
    fail_locked(error, retryable_, uncertain, now);
    return;
  }
  if (file_ == File::General && uncertain) {
    fail_locked("general metadata close unresolved", false, true, now);
    return;
  }
  // FTP close has already completed or failed; validation owns no active request.
  opened_ = false;
  validation_uncertain_ = uncertain;
  phase_ = Phase::Validate;
}

std::optional<Acquisition::MetadataWork> Acquisition::take_work_locked()
{
  if (phase_ != Phase::Cache && phase_ != Phase::Validate) {
    return std::nullopt;
  }
  MetadataWork work{++work_serial_, current_, file_, phase_ == Phase::Cache,
    validation_uncertain_, std::move(bytes_)};
  bytes_.clear();
  phase_ = Phase::Processing;
  return work;
}

Acquisition::MetadataResult Acquisition::validate_metadata(
  const MetadataWork & work, std::string_view bytes) const
{
  MetadataResult result;
  try {
    const auto & path = work.file.ftp_path;
    const bool compressed = path.size() >= 3 && path.substr(path.size() - 3) == ".xz";
    // General CRC covers uncompressed JSON; event CRC covers transferred bytes.
    if (work.role == File::Events && metadata_crc32(bytes) != work.file.crc) {
      result.error = "event metadata checksum mismatch";
      result.retryable = true;
      return result;
    }
    const auto json = Decoder::unpack_metadata(bytes, compressed);
    if (work.role == File::General) {
      if (metadata_crc32(json) != work.file.crc) {
        result.error = "general metadata checksum mismatch";
        result.retryable = true;
        return result;
      }
      result.events = event_metadata_file(json);
    } else {
      result.decoder = std::make_unique<Decoder>(report_error_);
      result.error = result.decoder->load_json(json);
    }
  } catch (const std::exception & ex) {
    result.error = ex.what();
  } catch (...) {
    result.error = "unknown metadata validation exception";
  }
  return result;
}

Acquisition::MetadataResult Acquisition::process_metadata(const MetadataWork & work) const
{
  MetadataResult result;
  try {
    const auto role = work.role == File::General ? MetadataRole::General : MetadataRole::Events;
    std::optional<std::string> cached;
    if (work.lookup) {
      cached = cache_->load(work.file, role);
      if (!cached) {
        result.cache_lookup_missed = true;
        return result;
      }
    }
    const std::string_view bytes = cached ? std::string_view(*cached) :
      std::string_view(work.bytes);
    result = validate_metadata(work, bytes);
    if (work.lookup && result.error.empty()) {
      RCLCPP_DEBUG(node_->get_logger(), "PX4 metadata cache hit: %s", work.file.uri.c_str());
    }
  } catch (const std::exception & ex) {
    result.error = ex.what();
  } catch (...) {
    result.error = "unknown metadata validation exception";
  }
  if (work.lookup && !result.error.empty()) {
    // Invalid cached content falls back to FTP without spending retry budget.
    result = {};
    result.cache_lookup_missed = true;
  }
  return result;
}

bool Acquisition::apply_metadata_locked(const MetadataWork & work, MetadataResult result)
{
  if (phase_ != Phase::Processing || work.serial != work_serial_) {
    return false;
  }
  if (result.cache_lookup_missed) {
    // End this cache lookup; downloaded bytes return later as separate validation work.
    begin_download_locked();
    return true;
  }
  if (!result.error.empty()) {
    fail_locked(result.error, result.retryable, work.uncertain, Clock::now());
    return true;
  }
  // Serialize publication with cancellation; content validation is already complete.
  if (cache_ && !work.lookup) {
    const auto role = work.role == File::General ? MetadataRole::General : MetadataRole::Events;
    const auto error = cache_->save_validated(work.file, role, work.bytes);
    if (!error.empty()) {
      RCLCPP_WARN(node_->get_logger(),
        "PX4 metadata cache save failed: %s; retaining validated metadata", error.c_str());
    }
  }
  if (work.role == File::General) {
    file_ = File::Events;
    begin_file_locked(*result.events);
  } else {
    finish_locked({std::move(result.decoder), {}, work.uncertain});
  }
  return true;
}

void Acquisition::process_work()
{
  // Cache access, validation, and persistence run in the existing callback group.
  // At most the general and event file can hit consecutively.
  while (true) {
    std::optional<MetadataWork> work;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      try {
        work = take_work_locked();
      } catch (const std::exception & ex) {
        stop_locked(ex.what(), true);
      } catch (...) {
        stop_locked("unknown metadata validation setup exception", true);
      }
    }
    if (!work) {
      return;
    }
    auto result = process_metadata(*work);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      try {
        // Cancellation may finish this operation and start a newer one during processing.
        if (!apply_metadata_locked(*work, std::move(result))) {
          continue;
        }
      } catch (const std::exception & ex) {
        fail_locked(ex.what(), false, work->uncertain, Clock::now());
      } catch (...) {
        fail_locked("unknown metadata validation exception", false, work->uncertain, Clock::now());
      }
    }
  }
}

void Acquisition::fail_locked(
  const std::string & reason, bool retryable, bool uncertain, Clock::time_point now)
{
  if (retryable && !uncertain && attempts_ < 3) {
    const auto backoff = std::chrono::seconds(attempts_ == 1 ? 2 : 5);
    ++attempts_;
    bytes_.clear();
    error_.clear();
    phase_ = Phase::Retry;
    next_request_ = now + backoff;
    RCLCPP_WARN(node_->get_logger(), "PX4 event metadata attempt failed: %s; retrying",
      reason.c_str());
    return;
  }
  finish_locked({nullptr, reason, uncertain});
}

void Acquisition::finish_locked(Result result)
{
  ++work_serial_;  // Invalidate metadata processing that may still be running.
  clear_pending_locked();
  phase_ = Phase::Idle;
  opened_ = false;
  stopping_ = false;
  stop_uncertain_ = false;
  timer_->cancel();
  bytes_.clear();
  error_.clear();
  if (!result.error.empty()) {
    RCLCPP_WARN(node_->get_logger(), "PX4 event acquisition failed: %s; retaining raw logging",
      result.error.c_str());
  }
  completion_ = Completion{std::move(callback_), std::move(result)};
}

void Acquisition::deliver()
{
  std::optional<Completion> completion;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    completion = std::move(completion_);
    completion_.reset();
  }
  if (completion) {
    try {
      completion->callback(std::move(completion->result));
    } catch (...) {
      RCLCPP_ERROR(node_->get_logger(), "PX4 event acquisition completion callback failed");
    }
  }
}

}  // namespace px4_events
}  // namespace mavros

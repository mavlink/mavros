// mavros
// Copyright 2026 Sean Hickey, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md

/**
 * @brief Atomic storage and bounded reuse of PX4 metadata
 * @file cache.cpp
 * @author Sean Hickey
 */

#include "cache.hpp"

#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

#include <libevents/parse/nlohmann/json.hpp>

#include "decoder.hpp"

namespace mavros
{
namespace px4_events
{
namespace
{
namespace fs = std::filesystem;
constexpr size_t MAX_HEADER_BYTES = 4096;

const char * role_name(MetadataRole role)
{
  return role == MetadataRole::General ? "general" : "events";
}

void validate(const MetadataFile & file, MetadataRole role, std::string_view bytes)
{
  const auto location = metadata_file(file.uri, file.crc);
  const bool compressed = location.ftp_path.size() >= 3 &&
    location.ftp_path.compare(location.ftp_path.size() - 3, 3, ".xz") == 0;
  const auto json = Decoder::unpack_metadata(bytes, compressed);
  if (metadata_crc32(role == MetadataRole::General ? std::string_view(json) : bytes) != file.crc) {
    throw std::runtime_error("cached metadata checksum mismatch");
  }
  if (role == MetadataRole::General) {
    event_metadata_file(json);
  } else {
    Decoder decoder;
    const auto error = decoder.load_json(json);
    if (!error.empty()) {
      throw std::runtime_error(error);
    }
  }
}

// Only our versioned final filenames are eligible for eviction. Temporary
// files, unrelated files, subdirectories, and symlinks are left alone.
bool managed_name(const std::string & name)
{
  if (name.size() != 28 || (name.substr(0, 5) != "v1-g-" && name.substr(0, 5) != "v1-e-") ||
    name[13] != '-' || name.substr(22) != ".cache")
  {
    return false;
  }
  const auto hex = [](char ch) {return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f');};
  return std::all_of(name.begin() + 5, name.begin() + 13, hex) &&
         std::all_of(name.begin() + 14, name.begin() + 22, hex);
}

class TemporaryEntry
{
public:
  explicit TemporaryEntry(const fs::path & directory)
  {
    const auto pattern = (directory / ".px4-events-XXXXXX").string();
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    fd_ = mkstemp(name.data());
    if (fd_ < 0) {
      throw std::system_error(errno, std::generic_category(), "create cache temporary file");
    }
    // Keep cleanup allocation-free after mkstemp succeeds.
    name_ = std::move(name);
  }

  ~TemporaryEntry()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    if (!name_.empty()) {
      ::unlink(name_.data());
    }
  }

  TemporaryEntry(const TemporaryEntry &) = delete;
  TemporaryEntry & operator=(const TemporaryEntry &) = delete;

  void publish(std::string_view bytes, const fs::path & destination)
  {
    while (!bytes.empty()) {
      const auto written = ::write(fd_, bytes.data(), bytes.size());
      if (written < 0 && errno == EINTR) {
        continue;
      }
      if (written <= 0) {
        throw std::system_error(
                written < 0 ? errno : EIO, std::generic_category(), "write cache entry");
      }
      bytes.remove_prefix(static_cast<size_t>(written));
    }
    const auto fd = std::exchange(fd_, -1);
    if (::close(fd) != 0) {
      throw std::system_error(errno, std::generic_category(), "close cache entry");
    }
    fs::rename(name_.data(), destination);
    name_.clear();
  }

private:
  int fd_{-1};
  std::vector<char> name_;
};
}  // namespace

MetadataCache::MetadataCache(fs::path directory, int64_t max_bytes)
: directory_(std::move(directory)), max_bytes_(max_bytes)
{
  if (max_bytes <= 0) {
    throw std::invalid_argument("metadata cache size limit must be positive");
  }
  if (directory_.empty()) {
    const auto home = std::getenv("HOME");
    if (!home || !*home) {
      throw std::runtime_error("HOME unavailable; configure an explicit metadata cache directory");
    }
    directory_ = fs::path(home) / ".cache/mavros/px4_events";
  }
}

fs::path MetadataCache::entry_path(const MetadataFile & file, MetadataRole role) const
{
  std::ostringstream name;
  // A filename hash collision is only a miss: load checks the full identity.
  name << (role == MetadataRole::General ? "v1-g-" : "v1-e-") << std::hex <<
    std::setfill('0') << std::setw(8) << metadata_crc32(file.uri) << '-' <<
    std::setw(8) << file.crc << ".cache";
  return directory_ / name.str();
}

std::optional<std::string> MetadataCache::load(const MetadataFile & file, MetadataRole role) const
{
  try {
    const auto path = entry_path(file, role);
    if (!fs::is_regular_file(fs::symlink_status(path))) {
      return std::nullopt;
    }
    const auto size = fs::file_size(path);
    if (size > max_bytes_ || size > Decoder::MAX_JSON_BYTES + MAX_HEADER_BYTES + 1) {
      return std::nullopt;
    }
    std::ifstream input(path, std::ios::binary);
    std::string entry(static_cast<size_t>(size), '\0');
    if (!input.read(entry.data(), static_cast<std::streamsize>(entry.size())) ||
      input.peek() != std::ifstream::traits_type::eof())
    {
      return std::nullopt;
    }
    const auto split = entry.find('\n');
    if (split == std::string::npos || split > MAX_HEADER_BYTES) {
      return std::nullopt;
    }
    const auto header = nlohmann::json::parse(entry.begin(), entry.begin() + split);
    if (header.at("version") != 1 || header.at("role") != role_name(role) ||
      header.at("uri") != file.uri || header.at("crc") != file.crc)
    {
      return std::nullopt;
    }
    auto bytes = entry.substr(split + 1);
    validate(file, role, bytes);
    return bytes;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::string MetadataCache::save(
  const MetadataFile & file, MetadataRole role, std::string_view bytes) const
{
  try {
    validate(file, role, bytes);
    const auto header = nlohmann::json{
      {"version", 1}, {"role", role_name(role)}, {"uri", file.uri}, {"crc", file.crc}}.dump();
    if (header.size() > MAX_HEADER_BYTES) {
      throw std::runtime_error("metadata cache identity exceeds header limit");
    }
    if (header.size() + 1 + bytes.size() > max_bytes_) {
      throw std::runtime_error("metadata cache entry exceeds configured cache limit");
    }
    fs::create_directories(directory_);
    TemporaryEntry temporary(directory_);
    const auto path = entry_path(file, role);
    temporary.publish(header + '\n' + std::string(bytes), path);
    evict(path);
    return {};
  } catch (const std::exception & error) {
    return error.what();
  }
}

void MetadataCache::evict(const fs::path & keep) const
{
  struct Entry
  {
    fs::path path;
    uintmax_t size;
    fs::file_time_type saved;
  };
  std::vector<Entry> entries;
  uintmax_t total = 0;
  for (const auto & item : fs::directory_iterator(directory_)) {
    std::error_code error;
    if (!managed_name(item.path().filename().string()) ||
      !fs::is_regular_file(item.symlink_status(error)) || error)
    {
      continue;
    }
    const auto size = item.file_size(error);
    if (error) {
      continue;
    }
    const auto saved = item.last_write_time(error);
    if (error) {
      continue;
    }
    if (size > std::numeric_limits<uintmax_t>::max() - total) {
      throw std::runtime_error("metadata cache size accounting overflow");
    }
    total += size;
    if (item.path() != keep) {
      entries.push_back({item.path(), size, saved});
    }
  }
  std::sort(entries.begin(), entries.end(), [](const Entry & a, const Entry & b) {
      return a.saved == b.saved ? a.path < b.path : a.saved < b.saved;
    });
  for (const auto & entry : entries) {
    if (total <= max_bytes_) {
      break;
    }
    std::error_code error;
    if (fs::remove(entry.path, error)) {
      total -= entry.size;
    }
    // Another instance may already have removed this file. Only successful
    // deletions reduce our snapshot estimate; a later save rechecks the bound.
  }
}

}  // namespace px4_events
}  // namespace mavros

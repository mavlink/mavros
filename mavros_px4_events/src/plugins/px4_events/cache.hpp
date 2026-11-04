// mavros
// Copyright 2026 Sean Hickey, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md

/**
 * @brief Persistent PX4 metadata cache
 * @file cache.hpp
 * @author Sean Hickey
 */

#ifndef PLUGINS__PX4_EVENTS__CACHE_HPP_
#define PLUGINS__PX4_EVENTS__CACHE_HPP_

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "metadata.hpp"

namespace mavros
{
namespace px4_events
{

enum class MetadataRole { General, Events };

/** No ROS/FTP ownership. Calls perform synchronous local file I/O.
 * Separate instances may share a directory; the disk bound is best effort.
 */
class MetadataCache
{
public:
  static constexpr int64_t DEFAULT_MAX_BYTES = 16 * 1024 * 1024;

  // Empty directory selects $HOME/.cache/mavros/px4_events. No directory is
  // created until save(). Invalid limits or an unavailable home throw.
  explicit MetadataCache(
    std::filesystem::path directory = {}, int64_t max_bytes = DEFAULT_MAX_BYTES);

  // Missing, unreadable, mismatched, or invalid entries are cache misses.
  // Returned bytes retain their original JSON/XZ representation.
  std::optional<std::string> load(const MetadataFile & file, MetadataRole role) const;

  // Revalidate before publication. Empty result means success; errors leave
  // the caller's in-memory definitions untouched. No interprocess lock.
  std::string save(const MetadataFile & file, MetadataRole role, std::string_view bytes) const;

private:
  std::filesystem::path directory_;
  uintmax_t max_bytes_;

  std::filesystem::path entry_path(const MetadataFile & file, MetadataRole role) const;
  void evict(const std::filesystem::path & keep) const;
};

}  // namespace px4_events
}  // namespace mavros

#endif  // PLUGINS__PX4_EVENTS__CACHE_HPP_

// Copyright 2026 Sean Hickey, All rights reserved.
// This file is part of MAVROS and subject to the top-level LICENSE.md.

#ifndef PX4_EVENTS_TEST_HELPERS_HPP_
#define PX4_EVENTS_TEST_HELPERS_HPP_

#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace mavros
{
namespace px4_events
{

class TemporaryMetadata
{
public:
  explicit TemporaryMetadata(const std::string & content, const std::string & suffix = ".json")
  {
    const auto pattern = "/tmp/mavros-px4-events-XXXXXX" + suffix;
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    const auto fd = mkstemps(name.data(), static_cast<int>(suffix.size()));
    if (fd < 0) {
      throw std::runtime_error("cannot create temporary event metadata");
    }
    close(fd);
    path = name.data();
    std::ofstream file(path, std::ios::binary);
    file.write(content.data(), static_cast<std::streamsize>(content.size()));
    if (!file) {
      std::remove(path.c_str());
      throw std::runtime_error("cannot write temporary event metadata");
    }
  }

  ~TemporaryMetadata()
  {
    std::remove(path.c_str());
  }

  TemporaryMetadata(const TemporaryMetadata &) = delete;
  TemporaryMetadata & operator=(const TemporaryMetadata &) = delete;

  std::string path;
};

}  // namespace px4_events
}  // namespace mavros

#endif  // PX4_EVENTS_TEST_HELPERS_HPP_

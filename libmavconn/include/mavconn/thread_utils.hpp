//
// libmavconn
// Copyright 2014,2015,2016,2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//
/**
 * @brief some useful utils
 * @file thread_utils.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavutils
 * @{
 *  @brief Some useful utils
 */

#pragma once
#ifndef MAVCONN__THREAD_UTILS_HPP_
#define MAVCONN__THREAD_UTILS_HPP_

#include <pthread.h>

#include <array>
#include <cstdio>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace mavconn
{
namespace utils
{

/**
 * @brief Make printf-formatted std::string
 *
 * Fast path formats into a small stack buffer with a single snprintf; only when
 * the output would not fit does it compute the exact size and re-format into a
 * heap string. Returns an empty string on an encoding error.
 */
template<typename ... Args>
std::string format(const std::string & fmt, Args... args)
{
  if constexpr (sizeof...(Args) == 0) {
    return fmt;
  } else {
    std::array<char, 256> stack{};
    const int written = std::snprintf(stack.data(), stack.size(), fmt.c_str(), args ...);
    if (written < 0) {
      return {};   // encoding error in the format or arguments
    }
    if (static_cast<size_t>(written) < stack.size()) {
      return std::string(stack.data(), written);
    }

    std::string ret;
    ret.resize(static_cast<size_t>(written));
    std::snprintf(ret.data(), ret.size() + 1, fmt.c_str(), args ...);
    return ret;
  }
}

/**
 * @brief Set name to current thread, printf-like
 * @param[in] name name for thread
 * @return true if success
 *
 * @note Only for Linux target
 * @todo add for other posix system
 */
template<typename ... Args>
bool set_this_thread_name(const std::string & name, Args && ... args)
{
  auto new_name = format(name, std::forward<Args>(args)...);

#ifdef __APPLE__
  return pthread_setname_np(new_name.c_str()) == 0;
#else
  pthread_t pth = pthread_self();
  return pthread_setname_np(pth, new_name.c_str()) == 0;
#endif
}

/**
 * @brief Convert to string objects with operator <<
 */
template<typename T>
inline const std::string to_string_ss(T & obj)
{
  std::ostringstream ss;
  ss << obj;
  return ss.str();
}

constexpr size_t operator""_KiB(unsigned long long sz)      // NOLINT
{
  return sz * 1024;
}

}  // namespace utils
}  // namespace mavconn

#endif  // MAVCONN__THREAD_UTILS_HPP_

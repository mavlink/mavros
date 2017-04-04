/**
 * @brief some useful utils
 * @file thread_utils.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavutils
 * @{
 *  @brief Some useful utils
 */
/*
 * libmavconn
 * Copyright 2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <thread>
#include <string>
#include <cstdio>
#include <sstream>
#include <pthread.h>

namespace mavconn {
namespace utils {

/**
 * @brief Make printf-formatted std::string
 *
 */
template<typename ... Args>
std::string format(const std::string &fmt, Args ... args)
{
	// C++11 specify that string store elements continously
	std::string ret;

	auto sz = std::snprintf(nullptr, 0, fmt.c_str(), args...);
	ret.reserve(sz + 1); ret.resize(sz);	// to be sure there have room for \0
	std::snprintf(&ret.front(), ret.capacity() + 1, fmt.c_str(), args...);
	return ret;
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
bool set_this_thread_name(const std::string &name, Args&& ... args)
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
template <typename T>
inline const std::string to_string_ss(T &obj)
{
	std::ostringstream ss;
	ss << obj;
	return ss.str();
}

constexpr size_t operator"" _KiB (unsigned long long sz)
{
	return sz * 1024;
}
}	// namespace utils
}	// namespace mavconn

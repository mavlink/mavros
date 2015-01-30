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
 * Copyright 2014 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#pragma once

#include <thread>
#include <cstdio>
#include <sstream>
#include <cstdarg>
#include <pthread.h>

namespace mavutils {

/**
 * @brief Set std::thread name with printf-like mode
 * @param[in] thd std::thread
 * @param[in] name name for thread
 * @return true if success
 *
 * @note Only for Linux target
 * @todo add for other posix system
 */
inline bool set_thread_name(std::thread &thd, const char *name, ...)
{
	pthread_t pth = thd.native_handle();
	va_list arg_list;
	va_start(arg_list, name);

	char new_name[256];
	vsnprintf(new_name, sizeof(new_name), name, arg_list);
	va_end(arg_list);
	return pthread_setname_np(pth, new_name) == 0;
}

/**
 * @brief Set thread name (std::string variation)
 */
template <typename Thread>
inline bool set_thread_name(Thread &thd, std::string &name)
{
	return set_thread_name(thd, name.c_str());
};

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

}; // namespace mavutils

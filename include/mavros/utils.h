/**
 * @brief some useful utils
 * @file utils.h
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

#include <algorithm>
#include <boost/thread/thread.hpp>

#include <cstdio>
#include <cstdarg>
#include <pthread.h>

#include <mavros/Mavlink.h>
#include <mavros/mavconn_mavlink.h>

namespace mavutils {

/**
 * @brief Set boost::thread name with printf-like mode
 * @param[in] thd boost::thread
 * @param[in] name name for thread
 * @return true if success
 *
 * @note Only for Linux target
 * @todo add for other posix system
 */
inline bool set_thread_name(boost::thread &thd, const char *name, ...)
{
	pthread_t pth = thd.native_handle();
	va_list arg_list;
	va_start(arg_list, name);

	char new_name[256];
	vsnprintf(new_name, sizeof(new_name), name, arg_list);
	va_end(arg_list);
	return pthread_setname_np(pth, new_name) == 0;
};

/**
 * @brief Set boost::thread name (std::string variation)
 */
inline bool set_thread_name(boost::thread &thd, std::string &name)
{
	return set_thread_name(thd, name.c_str());
};

/**
 * @brief Copy mavros/Mavlink.msg message data to mavlink_message_t
 */
inline bool copy_ros_to_mavlink(const mavros::Mavlink::ConstPtr &rmsg, mavlink_message_t &mmsg)
{
	if (rmsg->payload64.size() > sizeof(mmsg.payload64) / sizeof(mmsg.payload64[0])) {
		return false;
	}

	mmsg.msgid = rmsg->msgid;
	mmsg.len = rmsg->len;
	std::copy(rmsg->payload64.begin(), rmsg->payload64.end(), mmsg.payload64);
	return true;
};

/**
 * @brief Copy mavlink_message_t to mavros/Mavlink.msg
 */
inline void copy_mavlink_to_ros(const mavlink_message_t *mmsg, mavros::MavlinkPtr &rmsg)
{
	rmsg->len = mmsg->len;
	rmsg->seq = mmsg->seq;
	rmsg->sysid = mmsg->sysid;
	rmsg->compid = mmsg->compid;
	rmsg->msgid = mmsg->msgid;

	rmsg->payload64.reserve((mmsg->len + 7) / 8);
	for (size_t i = 0; i < (mmsg->len + 7) / 8; i++)
		rmsg->payload64.push_back(mmsg->payload64[i]);
};

}; // namespace mavutils

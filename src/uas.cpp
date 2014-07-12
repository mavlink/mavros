/**
 * @brief MAVROS UAS manager
 * @file uas.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
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

#include <mavros/mavros_uas.h>
#include <mavros/utils.h>

using namespace mavplugin;

UAS::UAS() :
	type(MAV_TYPE_GENERIC),
	autopilot(MAV_AUTOPILOT_GENERIC),
	target_system(1),
	target_component(1),
	connected(false),
	timer_service(),
	timer_work(new boost::asio::io_service::work(timer_service))
{
	// run io_service for async timers
	boost::thread t(boost::bind(&boost::asio::io_service::run, &this->timer_service));
	mavutils::set_thread_name(t, "TimerService");
	timer_thread.swap(t);
}

void UAS::stop(void)
{
	timer_work.reset();
	timer_service.stop();
}

void UAS::update_heartbeat(uint8_t type_, uint8_t autopilot_) {
	boost::recursive_mutex::scoped_lock lock(mutex);

	type = static_cast<enum MAV_TYPE>(type_);
	autopilot = static_cast<enum MAV_AUTOPILOT>(autopilot_);
}

void UAS::update_connection_status(bool conn_) {
	boost::recursive_mutex::scoped_lock lock(mutex);

	if (conn_ != connected) {
		connected = conn_;
		sig_connection_changed(connected);
	}
}


/**
 * @brief Mavlink diag class
 * @file mavlink_diag.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
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

#include <mavros/mavlink_diag.h>

using namespace mavros;

MavlinkDiag::MavlinkDiag(std::string name) :
	diagnostic_updater::DiagnosticTask(name),
	last_drop_count(0),
	is_connected(false)
{ };

void MavlinkDiag::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if (auto link = weak_link.lock()) {
		auto mav_status = link->get_status();
		auto iostat = link->get_iostat();

		stat.addf("Received packets:", "%u", mav_status.packet_rx_success_count);
		stat.addf("Dropped packets:", "%u", mav_status.packet_rx_drop_count);
		stat.addf("Buffer overruns:", "%u", mav_status.buffer_overrun);
		stat.addf("Parse errors:", "%u", mav_status.parse_error);
		stat.addf("Rx sequence number:", "%u", mav_status.current_rx_seq);
		stat.addf("Tx sequence number:", "%u", mav_status.current_tx_seq);

		stat.addf("Rx total bytes:", "%u", iostat.rx_total_bytes);
		stat.addf("Tx total bytes:", "%u", iostat.tx_total_bytes);
		stat.addf("Rx speed:", "%f", iostat.rx_speed);
		stat.addf("Tx speed:", "%f", iostat.tx_speed);

		if (mav_status.packet_rx_drop_count > last_drop_count)
			stat.summaryf(1, "%d packeges dropped since last report",
					mav_status.packet_rx_drop_count - last_drop_count);
		else if (is_connected)
			stat.summary(0, "connected");
		else
			// link operational, but not connected
			stat.summary(1, "not connected");

		last_drop_count = mav_status.packet_rx_drop_count;
	} else {
		stat.summary(2, "not connected");
	}
}


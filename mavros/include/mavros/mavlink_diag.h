/**
 * @brief Mavlink diag class
 * @file mavlink_diag.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
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

#pragma once

#include <diagnostic_updater/diagnostic_updater.h>
#include <mavconn/interface.h>

namespace mavros {

class MavlinkDiag : public diagnostic_updater::DiagnosticTask
{
public:
	explicit MavlinkDiag(std::string name);

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

	void set_mavconn(const mavconn::MAVConnInterface::Ptr &link) {
		weak_link = link;
	}

	void set_connection_status(bool connected) {
		is_connected = connected;
	}

private:
	mavconn::MAVConnInterface::WeakPtr weak_link;
	unsigned int last_drop_count;
	bool is_connected;
};

}; // namespace mavros


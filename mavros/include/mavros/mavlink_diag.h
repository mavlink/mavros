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
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
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
	std::atomic<bool> is_connected;
};
};	// namespace mavros


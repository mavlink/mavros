/**
 * @brief MAVLink all-dialect include helper
 * @file mavlink_undef.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#undef MAVLINK_H
#undef MAVLINK_STX

#undef MAVLINK_MESSAGE_CRCS

#undef MAVLINK_COMMON_H

// --- XXX: autogenerate this list ---

#undef HAVE_ENUM_MAV_AUTOPILOT
#undef HAVE_ENUM_MAV_TYPE
#undef HAVE_ENUM_MAV_CMD

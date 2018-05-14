/**
 * @brief MAVConn console-bridge compatibility header
 * @file console_bridge_compat.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2018 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <console_bridge/console.h>

// [[[cog:
// for idx, func in enumerate(('debug', 'inform', 'warn', 'error')):
//     fn = 'CONSOLE_BRIDGE_log{f}'.format(f=func.title())
//     fu = func.upper()
//
//     if func == 'inform':	# NOTE: special case
//         fu = 'INFO'
//
//     outl = lambda s: cog.outl(s.format(fn=fn, fu=fu))
//
//     if idx != 0:
//         outl('')
//
//     outl('#ifndef {fn}')
//     outl('#define {fn}(fmt, ...) \\')
//     outl('\tconsole_bridge::log(__FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_{fu}, fmt, ##__VA_ARGS__)')
//     outl('#endif // {fn}')
// ]]]
#ifndef CONSOLE_BRIDGE_logDebug
#define CONSOLE_BRIDGE_logDebug(fmt, ...) \
	console_bridge::log(__FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, fmt, ##__VA_ARGS__)
#endif // CONSOLE_BRIDGE_logDebug

#ifndef CONSOLE_BRIDGE_logInform
#define CONSOLE_BRIDGE_logInform(fmt, ...) \
	console_bridge::log(__FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_INFO, fmt, ##__VA_ARGS__)
#endif // CONSOLE_BRIDGE_logInform

#ifndef CONSOLE_BRIDGE_logWarn
#define CONSOLE_BRIDGE_logWarn(fmt, ...) \
	console_bridge::log(__FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_WARN, fmt, ##__VA_ARGS__)
#endif // CONSOLE_BRIDGE_logWarn

#ifndef CONSOLE_BRIDGE_logError
#define CONSOLE_BRIDGE_logError(fmt, ...) \
	console_bridge::log(__FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_ERROR, fmt, ##__VA_ARGS__)
#endif // CONSOLE_BRIDGE_logError
// [[[end]]] (checksum: c8dda3189b05a621b7244bf375275345)

/**
 * @brief MAVROS class
 * @file mavros.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros.h>
#include <ros/console.h>
#include <mavros/utils.h>
#include <fnmatch.h>

#ifdef MAVLINK_VERSION
#undef MAVLINK_VERSION
#endif
#include <mavlink/config.h>

using namespace mavros;
using namespace mavconn;
using namespace mavplugin;


MavRos::MavRos() :
	mavlink_nh("mavlink"),		// allow to namespace it
	fcu_link_diag("FCU connection"),
	gcs_link_diag("GCS bridge"),
	plugin_loader("mavros", "mavplugin::MavRosPlugin"),
	message_route_table {}
{
	std::string fcu_url, gcs_url;
	int system_id, component_id;
	int tgt_system_id, tgt_component_id;
	bool px4_usb_quirk;
	ros::V_string plugin_blacklist{}, plugin_whitelist{};
	MAVConnInterface::Ptr fcu_link;

	ros::NodeHandle nh("~");

	nh.param<std::string>("fcu_url", fcu_url, "serial:///dev/ttyACM0");
	nh.param<std::string>("gcs_url", gcs_url, "udp://@");
	nh.param("system_id", system_id, 1);
	nh.param<int>("component_id", component_id, MAV_COMP_ID_UDP_BRIDGE);
	nh.param("target_system_id", tgt_system_id, 1);
	nh.param("target_component_id", tgt_component_id, 1);
	nh.param("startup_px4_usb_quirk", px4_usb_quirk, false);
	nh.getParam("plugin_blacklist", plugin_blacklist);
	nh.getParam("plugin_whitelist", plugin_whitelist);

	// Now we use FCU URL as a hardware Id
	UAS_DIAG(&mav_uas).setHardwareID(fcu_url);

	ROS_INFO_STREAM("FCU URL: " << fcu_url);
	try {
		fcu_link = MAVConnInterface::open_url(fcu_url, system_id, component_id);
		// may be overridden by URL
		system_id = fcu_link->get_system_id();
		component_id = fcu_link->get_component_id();

		fcu_link_diag.set_mavconn(fcu_link);
		UAS_DIAG(&mav_uas).add(fcu_link_diag);
	}
	catch (mavconn::DeviceError &ex) {
		ROS_FATAL("FCU: %s", ex.what());
		ros::shutdown();
		return;
	}

	if (gcs_url != "") {
		ROS_INFO_STREAM("GCS URL: " << gcs_url);
		try {
			gcs_link = MAVConnInterface::open_url(gcs_url, system_id, component_id);

			gcs_link_diag.set_mavconn(gcs_link);
			UAS_DIAG(&mav_uas).add(gcs_link_diag);
		}
		catch (mavconn::DeviceError &ex) {
			ROS_FATAL("GCS: %s", ex.what());
			ros::shutdown();
			return;
		}
	}
	else
		ROS_INFO("GCS bridge disabled");

	// ROS mavlink bridge
	mavlink_pub = mavlink_nh.advertise<mavros_msgs::Mavlink>("from", 100);
	mavlink_sub = mavlink_nh.subscribe("to", 100, &MavRos::mavlink_sub_cb, this,
		ros::TransportHints()
			.unreliable().maxDatagramSize(1024)
			.reliable());

	// setup UAS and diag
	mav_uas.set_tgt(tgt_system_id, tgt_component_id);
	UAS_FCU(&mav_uas) = fcu_link;
	mav_uas.sig_connection_changed.connect(boost::bind(&MavlinkDiag::set_connection_status, &fcu_link_diag, _1));
	mav_uas.sig_connection_changed.connect(boost::bind(&MavRos::log_connect_change, this, _1));

	// connect FCU link
	fcu_link->message_received.connect(boost::bind(&MavRos::mavlink_pub_cb, this, _1, _2, _3));
	fcu_link->message_received.connect(boost::bind(&MavRos::plugin_route_cb, this, _1, _2, _3));
	fcu_link->port_closed.connect(boost::bind(&MavRos::terminate_cb, this));

	if (gcs_link) {
		// setup GCS link bridge
		fcu_link->message_received.connect(
			boost::bind(&MAVConnInterface::send_message, gcs_link, _1, _2, _3));
		gcs_link->message_received.connect(
			boost::bind(&MAVConnInterface::send_message, fcu_link, _1, _2, _3));
		gcs_link_diag.set_connection_status(true);
	}

	// prepare plugin lists
	// issue #257 2: assume that all plugins blacklisted
	if (plugin_blacklist.empty() and !plugin_whitelist.empty())
		plugin_blacklist.push_back("*");

	for (auto &name : plugin_loader.getDeclaredClasses())
		add_plugin(name, plugin_blacklist, plugin_whitelist);

	if (px4_usb_quirk)
		startup_px4_usb_quirk();

#define STR2(x)	#x
#define STR(x)	STR2(x)

	ROS_INFO("Built-in SIMD instructions: %s", Eigen::SimdInstructionSetsInUse());
	ROS_INFO("Built-in MAVLink package version: %s", MAVLINK_VERSION);
	ROS_INFO("Built-in MAVLink dialect: %s", STR(MAVLINK_DIALECT));
	ROS_INFO("MAVROS started. MY ID %d.%d, TARGET ID %d.%d",
		system_id, component_id,
		tgt_system_id, tgt_component_id);
}

void MavRos::spin() {
	ros::AsyncSpinner spinner(4 /* threads */);

	auto diag_timer = mavlink_nh.createTimer(
			ros::Duration(0.5),
			[&](const ros::TimerEvent &) {
				UAS_DIAG(&mav_uas).update();
			});
	diag_timer.start();

	spinner.start();
	ros::waitForShutdown();

	ROS_INFO("Stopping mavros...");
	mav_uas.stop();
	spinner.stop();
}

void MavRos::mavlink_pub_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
	auto rmsg = boost::make_shared<mavros_msgs::Mavlink>();

	if  (mavlink_pub.getNumSubscribers() == 0)
		return;

	rmsg->header.stamp = ros::Time::now();
	mavros_msgs::mavlink::convert(*mmsg, *rmsg);
	mavlink_pub.publish(rmsg);
}

void MavRos::mavlink_sub_cb(const mavros_msgs::Mavlink::ConstPtr &rmsg) {
	mavlink_message_t mmsg;

	if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
		UAS_FCU(&mav_uas)->send_message(&mmsg, rmsg->sysid, rmsg->compid);
	else
		ROS_ERROR("Drop mavlink packet: illegal payload64 size");
}

void MavRos::plugin_route_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
	message_route_table[mmsg->msgid](mmsg, sysid, compid);
}

static bool pattern_match(std::string &pattern, std::string &pl_name) {
	int cmp = fnmatch(pattern.c_str(), pl_name.c_str(), FNM_CASEFOLD);
	if (cmp == 0)
		return true;
	else if (cmp != FNM_NOMATCH) {
		// never see that, i think that it is fatal error.
		ROS_FATAL("Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
				pattern.c_str(), pl_name.c_str(), cmp);
		ros::shutdown();
	}

	return false;
}

/**
 * @brief Checks that plugin blacklisted
 *
 * Operation algo:
 *
 *  1. if blacklist and whitelist is empty: load all
 *  2. if blacklist is empty and whitelist non empty: assume blacklist is ["*"]
 *  3. if blacklist non empty: usual blacklist behavior
 *  4. if whitelist non empty: override blacklist
 *
 * @note Issue #257.
 */
bool MavRos::is_blacklisted(std::string &pl_name, ros::V_string &blacklist, ros::V_string &whitelist) {
	for (auto &bl_pattern : blacklist) {
		if (pattern_match(bl_pattern, pl_name)) {
			for (auto &wl_pattern : whitelist) {
				if (pattern_match(wl_pattern, pl_name))
					return false;
			}

			return true;
		}
	}

	return false;
}

/**
 * @brief Loads plugin (if not blacklisted)
 */
void MavRos::add_plugin(std::string &pl_name, ros::V_string &blacklist, ros::V_string &whitelist) {
	if (is_blacklisted(pl_name, blacklist, whitelist)) {
		ROS_INFO_STREAM("Plugin " << pl_name << " blacklisted");
		return;
	}

	try {
		auto plugin = plugin_loader.createInstance(pl_name);
		plugin->initialize(mav_uas);
		loaded_plugins.push_back(plugin);

		ROS_INFO_STREAM("Plugin " << pl_name << " loaded and initialized");

		for (auto &pair : plugin->get_rx_handlers()) {
			ROS_DEBUG_STREAM("Route msgid " << int(pair.first) << " to " << pl_name);
			message_route_table[pair.first].connect(pair.second);
		}
	} catch (pluginlib::PluginlibException &ex) {
		ROS_ERROR_STREAM("Plugin " << pl_name << " load exception: " << ex.what());
	}
}

void MavRos::terminate_cb() {
	ROS_ERROR("FCU connection closed, mavros will be terminated.");
	ros::requestShutdown();
}

void MavRos::startup_px4_usb_quirk(void) {
	/* sample code from QGC */
	const uint8_t init[] = {0x0d, 0x0d, 0x0d, 0};
	const uint8_t nsh[] = "sh /etc/init.d/rc.usb\n";

	ROS_INFO("Autostarting mavlink via USB on PX4");
	UAS_FCU(&mav_uas)->send_bytes(init, 3);
	UAS_FCU(&mav_uas)->send_bytes(nsh, sizeof(nsh) - 1);
	UAS_FCU(&mav_uas)->send_bytes(init, 4);	/* NOTE in original init[3] */
}

void MavRos::log_connect_change(bool connected) {
	auto ap = mav_uas.str_autopilot(mav_uas.get_autopilot());

	/* note: sys_status plugin required */
	if (connected)
		ROS_INFO("CON: Got HEARTBEAT, connected. FCU: %s", ap.c_str());
	else
		ROS_WARN("CON: Lost connection, HEARTBEAT timed out.");
}


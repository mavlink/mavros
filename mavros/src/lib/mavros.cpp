/**
 * @brief MAVROS class
 * @file mavros.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2013,2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros.h>
#include <ros/console.h>
#include <mavros/utils.h>
#include <fnmatch.h>

using namespace mavros;
using namespace mavconn;
using namespace mavplugin;


MavRos::MavRos() :
	mavlink_nh("/mavlink"),		// for compatible reasons
	fcu_link_diag("FCU connection"),
	gcs_link_diag("GCS bridge"),
	plugin_loader("mavros", "mavplugin::MavRosPlugin"),
	message_route_table {}
{
	std::string fcu_url, gcs_url;
	int system_id, component_id;
	int tgt_system_id, tgt_component_id;
	bool px4_usb_quirk;
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
	mavlink_pub = mavlink_nh.advertise<Mavlink>("from", 100);
	mavlink_sub = mavlink_nh.subscribe("to", 100, &MavRos::mavlink_sub_cb, this,
		ros::TransportHints()
			.unreliable()
			.maxDatagramSize(1024));

	mav_uas.set_tgt(tgt_system_id, tgt_component_id);
	UAS_FCU(&mav_uas) = fcu_link;
	mav_uas.sig_connection_changed.connect(boost::bind(&MavlinkDiag::set_connection_status, &fcu_link_diag, _1));
	mav_uas.sig_connection_changed.connect(boost::bind(&MavRos::log_connect_change, this, _1));

	fcu_link->message_received.connect(boost::bind(&MavRos::mavlink_pub_cb, this, _1, _2, _3));
	fcu_link->message_received.connect(boost::bind(&MavRos::plugin_route_cb, this, _1, _2, _3));
	fcu_link->port_closed.connect(boost::bind(&MavRos::terminate_cb, this));

	if (gcs_link) {
		fcu_link->message_received.connect(
			boost::bind(&MAVConnInterface::send_message, gcs_link, _1, _2, _3));
		gcs_link->message_received.connect(
			boost::bind(&MAVConnInterface::send_message, fcu_link, _1, _2, _3));
		gcs_link_diag.set_connection_status(true);
	}

	for (auto &name : plugin_loader.getDeclaredClasses())
		add_plugin(name);

	if (px4_usb_quirk)
		startup_px4_usb_quirk();

#define STR2(x)	#x
#define STR(x)	STR2(x)

	ROS_INFO("Built-in mavlink dialect: %s", STR(MAVLINK_DIALECT));
	ROS_INFO("MAVROS started. MY ID [%d, %d], TARGET ID [%d, %d]",
		system_id, component_id,
		tgt_system_id, tgt_component_id);
}

void MavRos::spin() {
	ros::AsyncSpinner spinner(4 /* threads */);

	spinner.start();

	ros::Rate loop_rate(1000);
	while (ros::ok()) {
		UAS_DIAG(&mav_uas).update();
		loop_rate.sleep();
	}

	ROS_INFO("Stopping mavros...");
	mav_uas.stop();
}

void MavRos::mavlink_pub_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
	auto rmsg = boost::make_shared<Mavlink>();

	if  (mavlink_pub.getNumSubscribers() == 0)
		return;

	rmsg->header.stamp = ros::Time::now();
	mavutils::copy_mavlink_to_ros(mmsg, rmsg);
	mavlink_pub.publish(rmsg);
}

void MavRos::mavlink_sub_cb(const Mavlink::ConstPtr &rmsg) {
	mavlink_message_t mmsg;

	if (mavutils::copy_ros_to_mavlink(rmsg, mmsg))
		UAS_FCU(&mav_uas)->send_message(&mmsg, rmsg->sysid, rmsg->compid);
	else
		ROS_ERROR("Drop mavlink packet: illegal payload64 size");
}

void MavRos::plugin_route_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
	// Ignore messages not sent by the target UAS
	if (mav_uas.is_sender(sysid, compid)) {
		message_route_table[mmsg->msgid](mmsg, sysid, compid);
	}
}

bool MavRos::check_in_blacklist(std::string &pl_name) {
	for (auto &pattern : plugin_blacklist) {
		int cmp = fnmatch(pattern.c_str(), pl_name.c_str(), FNM_CASEFOLD);
		if (cmp == 0)
			return true;
		else if (cmp != FNM_NOMATCH)
			ROS_ERROR("Blacklist check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
				pattern.c_str(), pl_name.c_str(), cmp);
	}

	return false;
}

void MavRos::add_plugin(std::string &pl_name) {
	if (check_in_blacklist(pl_name)) {
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
	/* note: sys_status plugin required */
	if (connected)
		ROS_INFO("CON: Got HEARTBEAT, connected.");
	else
		ROS_WARN("CON: Lost connection, HEARTBEAT timed out.");
}

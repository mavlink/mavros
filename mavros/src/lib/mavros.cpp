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

#include <ros/console.h>
#include <mavros/mavros.h>
#include <mavros/utils.h>
#include <fnmatch.h>

// MAVLINK_VERSION string
#include <mavlink/config.h>
#include <mavconn/mavlink_dialect.h>

#include <mavconn/udp.h>
#include <std_msgs/String.h>

using namespace mavros;
using mavconn::MAVConnInterface;
using mavconn::Framing;
using mavlink::mavlink_message_t;
using plugin::PluginBase;
using utils::enum_value;


MavRos::MavRos() :
	mavlink_nh("mavlink"),		// allow to namespace it
	fcu_link_diag("FCU connection"),
	gcs_link_diag("GCS bridge"),
	plugin_loader("mavros", "mavros::plugin::PluginBase"),
	last_message_received_from_gcs(0),
	plugin_subscriptions{}
{
	std::string fcu_url, gcs_url;
	std::string fcu_protocol;
	int system_id, component_id;
	int tgt_system_id, tgt_component_id;
	bool px4_usb_quirk;
	double conn_timeout_d;
	ros::V_string plugin_blacklist{}, plugin_whitelist{};
	MAVConnInterface::Ptr fcu_link;

	ros::NodeHandle nh("~");

	nh.param<std::string>("fcu_url", fcu_url, "serial:///dev/ttyACM0");
	nh.param<std::string>("gcs_url", gcs_url, "udp://@");
	nh.param<bool>("gcs_quiet_mode", gcs_quiet_mode, false);
	nh.param("conn/timeout", conn_timeout_d, 30.0);

	nh.param<std::string>("fcu_protocol", fcu_protocol, "v2.0");
	nh.param("system_id", system_id, 1);
	nh.param<int>("component_id", component_id, mavconn::MAV_COMP_ID_UDP_BRIDGE);
	nh.param("target_system_id", tgt_system_id, 1);
	nh.param("target_component_id", tgt_component_id, 1);
	nh.param("startup_px4_usb_quirk", px4_usb_quirk, false);
	nh.getParam("plugin_blacklist", plugin_blacklist);
	nh.getParam("plugin_whitelist", plugin_whitelist);

	conn_timeout = ros::Duration(conn_timeout_d);

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

	if (fcu_protocol == "v1.0") {
		fcu_link->set_protocol_version(mavconn::Protocol::V10);
	}
	else if (fcu_protocol == "v2.0") {
		fcu_link->set_protocol_version(mavconn::Protocol::V20);
	}
	//else if (fcu_protocol == "auto") {	// XXX TODO
	//	fcu_link->set_protocol_version(mavconn::Protocol::V20);
	//}
	else {
		ROS_WARN("Unknown FCU protocol: \"%s\", should be: \"v1.0\" or \"v2.0\". Used default v1.0.", fcu_protocol.c_str());
		fcu_link->set_protocol_version(mavconn::Protocol::V10);
	}

	if (gcs_url != "") {
		ROS_INFO_STREAM("GCS URL: " << gcs_url);
		try {
			gcs_link = MAVConnInterface::open_url(gcs_url, system_id, component_id);

			gcs_link_diag.set_mavconn(gcs_link);
			gcs_diag_updater.setHardwareID(gcs_url);
			gcs_diag_updater.add(gcs_link_diag);
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

	mav_uas.add_connection_change_handler(std::bind(&MavlinkDiag::set_connection_status, &fcu_link_diag, std::placeholders::_1));
	mav_uas.add_connection_change_handler(std::bind(&MavRos::log_connect_change, this, std::placeholders::_1));

	// prepare plugin lists
	// issue #257 2: assume that all plugins blacklisted
	if (plugin_blacklist.empty() and !plugin_whitelist.empty())
		plugin_blacklist.emplace_back("*");

	for (auto &name : plugin_loader.getDeclaredClasses())
		add_plugin(name, plugin_blacklist, plugin_whitelist);

	// connect FCU link

	// XXX TODO: move workers to ROS Spinner, let mavconn threads to do only IO
	fcu_link->message_received_cb = [this](const mavlink_message_t *msg, const Framing framing) {
		mavlink_pub_cb(msg, framing);
		plugin_route_cb(msg, framing);

		if (gcs_link) {
			if (this->gcs_quiet_mode && msg->msgid != mavlink::minimal::msg::HEARTBEAT::MSG_ID &&
				(ros::Time::now() - this->last_message_received_from_gcs > this->conn_timeout)) {
				return;
			}

			gcs_link->send_message_ignore_drop(msg);
		}
	};

	fcu_link->port_closed_cb = []() {
		ROS_ERROR("FCU connection closed, mavros will be terminated.");
		ros::requestShutdown();
	};

	if (gcs_link) {
		// setup GCS link bridge
		gcs_link->message_received_cb = [this, fcu_link](const mavlink_message_t *msg, const Framing framing) {
			this->last_message_received_from_gcs = ros::Time::now();
			fcu_link->send_message_ignore_drop(msg);
		};

		gcs_link_diag.set_connection_status(true);
	}

	if (px4_usb_quirk)
		startup_px4_usb_quirk();

	std::stringstream ss;
	for (auto &s : mavconn::MAVConnInterface::get_known_dialects())
		ss << " " << s;

	ROS_INFO("Built-in SIMD instructions: %s", Eigen::SimdInstructionSetsInUse());
	ROS_INFO("Built-in MAVLink package version: %s", MAVLINK_VERSION);
	ROS_INFO("Known MAVLink dialects:%s", ss.str().c_str());
	ROS_INFO("MAVROS started. MY ID %u.%u, TARGET ID %u.%u",
		system_id, component_id,
		tgt_system_id, tgt_component_id);
}

void MavRos::spin()
{
	ros::AsyncSpinner spinner(4 /* threads */);

	auto diag_timer = mavlink_nh.createTimer(
			ros::Duration(0.5),
			[&](const ros::TimerEvent &) {
				UAS_DIAG(&mav_uas).update();

				if (gcs_link)
					gcs_diag_updater.update();
			});
	diag_timer.start();

	auto remote_endpoint_timer = mavlink_nh.createTimer(
		ros::Duration(1.0), [&](const ros::TimerEvent&) {
			static auto pub = mavlink_nh.advertise<std_msgs::String>("gcs_ip", 1, true /* latch */);
			static auto finished = false;

			if (finished)
				return;

			if (const auto* p = dynamic_cast<mavconn::MAVConnUDP*>(gcs_link.get())) {
				const auto endpoint = p->get_remote_endpoint();
				const auto endpoint_valid = endpoint.find("255.255.255.255") == std::string::npos;

				if (endpoint_valid) {
					std_msgs::String msg;
					msg.data = endpoint.substr(0, endpoint.find(":"));
					pub.publish(msg);
					finished = true;
				}
			}
		});
	remote_endpoint_timer.start();

	spinner.start();
	ros::waitForShutdown();

	ROS_INFO("Stopping mavros...");
	spinner.stop();
}

void MavRos::mavlink_pub_cb(const mavlink_message_t *mmsg, Framing framing)
{
	auto rmsg = boost::make_shared<mavros_msgs::Mavlink>();

	if  (mavlink_pub.getNumSubscribers() == 0)
		return;

	rmsg->header.stamp = ros::Time::now();
	mavros_msgs::mavlink::convert(*mmsg, *rmsg, enum_value(framing));
	mavlink_pub.publish(rmsg);
}

void MavRos::mavlink_sub_cb(const mavros_msgs::Mavlink::ConstPtr &rmsg)
{
	mavlink_message_t mmsg;

	if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
		UAS_FCU(&mav_uas)->send_message_ignore_drop(&mmsg);
	else
		ROS_ERROR("Drop mavlink packet: convert error.");
}

void MavRos::plugin_route_cb(const mavlink_message_t *mmsg, const Framing framing)
{
	auto it = plugin_subscriptions.find(mmsg->msgid);
	if (it == plugin_subscriptions.end())
		return;

	for (auto &info : it->second)
		std::get<3>(info)(mmsg, framing);
}

static bool pattern_match(std::string &pattern, std::string &pl_name)
{
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
static bool is_blacklisted(std::string &pl_name, ros::V_string &blacklist, ros::V_string &whitelist)
{
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

inline bool is_mavlink_message_t(const size_t rt)
{
	static const auto h = typeid(mavlink_message_t).hash_code();
	return h == rt;
}

/**
 * @brief Loads plugin (if not blacklisted)
 */
void MavRos::add_plugin(std::string &pl_name, ros::V_string &blacklist, ros::V_string &whitelist)
{
	if (is_blacklisted(pl_name, blacklist, whitelist)) {
		ROS_INFO_STREAM("Plugin " << pl_name << " blacklisted");
		return;
	}

	try {
		auto plugin = plugin_loader.createInstance(pl_name);

		ROS_INFO_STREAM("Plugin " << pl_name << " loaded");

		for (auto &info : plugin->get_subscriptions()) {
			auto msgid = std::get<0>(info);
			auto msgname = std::get<1>(info);
			auto type_hash_ = std::get<2>(info);

			std::string log_msgname;

			if (is_mavlink_message_t(type_hash_))
				log_msgname = utils::format("MSG-ID (%u) <%zu>", msgid, type_hash_);
			else
				log_msgname = utils::format("%s (%u) <%zu>", msgname, msgid, type_hash_);

			ROS_DEBUG_STREAM("Route " << log_msgname << " to " << pl_name);

			auto it = plugin_subscriptions.find(msgid);
			if (it == plugin_subscriptions.end()) {
				// new entry

				ROS_DEBUG_STREAM(log_msgname << " - new element");
				plugin_subscriptions[msgid] = PluginBase::Subscriptions{{info}};
			}
			else {
				// existing: check handler message type

				bool append_allowed = is_mavlink_message_t(type_hash_);
				if (!append_allowed) {
					append_allowed = true;
					for (auto &e : it->second) {
						auto t2 = std::get<2>(e);
						if (!is_mavlink_message_t(t2) && t2 != type_hash_) {
							ROS_ERROR_STREAM(log_msgname << " routed to different message type (hash: " << t2 << ")");
							append_allowed = false;
						}
					}
				}

				if (append_allowed) {
					ROS_DEBUG_STREAM(log_msgname << " - emplace");
					it->second.emplace_back(info);
				}
				else
					ROS_ERROR_STREAM(log_msgname << " handler dropped because this ID are used for another message type");
			}
		}

		plugin->initialize(mav_uas);
		loaded_plugins.push_back(plugin);

		ROS_INFO_STREAM("Plugin " << pl_name << " initialized");
	} catch (pluginlib::PluginlibException &ex) {
		ROS_ERROR_STREAM("Plugin " << pl_name << " load exception: " << ex.what());
	}
}

void MavRos::startup_px4_usb_quirk()
{
       /* sample code from QGC */
       const uint8_t init[] = {0x0d, 0x0d, 0x0d, 0};
       const uint8_t nsh[] = "sh /etc/init.d/rc.usb\n";

       ROS_INFO("Autostarting mavlink via USB on PX4");
       UAS_FCU(&mav_uas)->send_bytes(init, 3);
       UAS_FCU(&mav_uas)->send_bytes(nsh, sizeof(nsh) - 1);
       UAS_FCU(&mav_uas)->send_bytes(init, 4); /* NOTE in original init[3] */
}

void MavRos::log_connect_change(bool connected)
{
	auto ap = utils::to_string(mav_uas.get_autopilot());

	/* note: sys_status plugin required */
	if (connected)
		ROS_INFO("CON: Got HEARTBEAT, connected. FCU: %s", ap.c_str());
	else
		ROS_WARN("CON: Lost connection, HEARTBEAT timed out.");
}

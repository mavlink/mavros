/**
 * @brief MAVROS Node
 * @file mavros_node.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2013 Vladimir Ermakov.
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

#include <ros/ros.h>
#include <ros/console.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <mavros/mavconn_interface.h>

#include <pluginlib/class_loader.h>
#include <mavros/mavros_plugin.h>
#include <mavros/utils.h>
#include <fnmatch.h>

#include <mavros/Mavlink.h>

using namespace mavros;
using namespace mavconn;
using namespace mavplugin;


class MavlinkDiag : public diagnostic_updater::DiagnosticTask
{
public:
	explicit MavlinkDiag(std::string name) :
		diagnostic_updater::DiagnosticTask(name),
		last_drop_count(0),
		is_connected(false)
	{};

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		if (boost::shared_ptr<MAVConnInterface> link = weak_link.lock()) {
			mavlink_status_t mav_status = link->get_status();

			stat.addf("Received packets:", "%u", mav_status.packet_rx_success_count);
			stat.addf("Dropped packets:", "%u", mav_status.packet_rx_drop_count);
			stat.addf("Buffer overruns:", "%u", mav_status.buffer_overrun);
			stat.addf("Parse errors:", "%u", mav_status.parse_error);
			stat.addf("Rx sequence number:", "%u", mav_status.current_rx_seq);
			stat.addf("Tx sequence number:", "%u", mav_status.current_tx_seq);

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

	void set_mavconn(const boost::shared_ptr<MAVConnInterface> &link) {
		weak_link = link;
	}

	void set_connection_status(bool connected) {
		is_connected = connected;
	}

private:
	boost::weak_ptr<MAVConnInterface> weak_link;
	unsigned int last_drop_count;
	bool is_connected;
};


class MavRos
{
public:
	explicit MavRos(const ros::NodeHandle &nh_) :
		node_handle(nh_),
		mavlink_node_handle("/mavlink"), // for compatible reasons
		fcu_link_diag("FCU connection"),
		gcs_link_diag("GCS bridge"),
		plugin_loader("mavros", "mavplugin::MavRosPlugin"),
		message_route_table(256)
	{
		std::string fcu_url, gcs_url;
		int system_id, component_id;
		int tgt_system_id, tgt_component_id;
		bool px4_usb_quirk;
		boost::shared_ptr<MAVConnInterface> fcu_link;

		node_handle.param<std::string>("fcu_url", fcu_url, "serial:///dev/ttyACM0");
		node_handle.param<std::string>("gcs_url", gcs_url, "udp://@");
		node_handle.param("system_id", system_id, 1);
		node_handle.param<int>("component_id", component_id, MAV_COMP_ID_UDP_BRIDGE);
		node_handle.param("target_system_id", tgt_system_id, 1);
		node_handle.param("target_component_id", tgt_component_id, 1);
		node_handle.param("startup_px4_usb_quirk", px4_usb_quirk, false);
		node_handle.getParam("plugin_blacklist", plugin_blacklist);

		diag_updater.setHardwareID("Mavlink");
		diag_updater.add(fcu_link_diag);

		ROS_INFO_STREAM("FCU URL: " << fcu_url);
		fcu_link = MAVConnInterface::open_url(fcu_url, system_id, component_id);
		// may be overridden by URL
		system_id = fcu_link->get_system_id();
		component_id = fcu_link->get_component_id();

		if (gcs_url != "") {
			ROS_INFO_STREAM("GCS URL: " << gcs_url);
			diag_updater.add(gcs_link_diag);
			gcs_link = MAVConnInterface::open_url(gcs_url, system_id, component_id);
		}
		else
			ROS_INFO("GCS bridge disabled");

		mavlink_pub = mavlink_node_handle.advertise<Mavlink>("from", 100);

		if (gcs_link)
			fcu_link->message_received.connect(
					boost::bind(&MAVConnInterface::send_message, gcs_link, _1, _2, _3));

		fcu_link->message_received.connect(boost::bind(&MavRos::mavlink_pub_cb, this, _1, _2, _3));
		fcu_link->message_received.connect(boost::bind(&MavRos::plugin_route_cb, this, _1, _2, _3));
		fcu_link->port_closed.connect(boost::bind(&MavRos::terminate_cb, this));
		fcu_link_diag.set_mavconn(fcu_link);

		mavlink_sub = mavlink_node_handle.subscribe("to", 100, &MavRos::mavlink_sub_cb, this,
				ros::TransportHints()
					.unreliable()
					.maxDatagramSize(1024));

		if (gcs_link) {
			gcs_link->message_received.connect(
					boost::bind(&MAVConnInterface::send_message, fcu_link, _1, _2, _3));
			gcs_link_diag.set_mavconn(gcs_link);
			gcs_link_diag.set_connection_status(true);
		}

		mav_uas.set_tgt(tgt_system_id, tgt_component_id);
		UAS_FCU(&mav_uas) = fcu_link;
		mav_uas.sig_connection_changed.connect(boost::bind(&MavlinkDiag::set_connection_status, &fcu_link_diag, _1));
		mav_uas.sig_connection_changed.connect(boost::bind(&MavRos::log_connect_change, this, _1));

		auto plugins = plugin_loader.getDeclaredClasses();
		loaded_plugins.reserve(plugins.size());
		for (auto it = plugins.begin();
				it != plugins.end();
				++it)
			add_plugin(*it);

		if (px4_usb_quirk)
			startup_px4_usb_quirk();

		ROS_INFO("MAVROS started. MY ID [%d, %d], TARGET ID [%d, %d]",
				system_id, component_id,
				tgt_system_id, tgt_component_id);
	}

	~MavRos() {};

	void spin() {
		ros::Rate loop_rate(1000);
		while (node_handle.ok()) {
			ros::spinOnce();
			diag_updater.update();

			loop_rate.sleep();
		}

		ROS_INFO("Stopping mavros...");
		mav_uas.stop();
	}

private:
	ros::NodeHandle node_handle;
	ros::NodeHandle mavlink_node_handle;
	// fcu_link stored in mav_uas
	boost::shared_ptr<MAVConnInterface> gcs_link;

	ros::Publisher mavlink_pub;
	ros::Subscriber mavlink_sub;

	diagnostic_updater::Updater diag_updater;
	MavlinkDiag fcu_link_diag;
	MavlinkDiag gcs_link_diag;

	pluginlib::ClassLoader<mavplugin::MavRosPlugin> plugin_loader;
	std::vector<boost::shared_ptr<MavRosPlugin> > loaded_plugins;
	std::vector<std::string> plugin_blacklist;
	std::vector<sig2::signal<void(const mavlink_message_t *message, uint8_t system_id, uint8_t component_id)> >
		message_route_table; // link interface -> router -> plugin callback
	UAS mav_uas;

	void mavlink_pub_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
		MavlinkPtr rmsg = boost::make_shared<Mavlink>();

		if  (mavlink_pub.getNumSubscribers() == 0)
			return;

		rmsg->header.stamp = ros::Time::now();
		mavutils::copy_mavlink_to_ros(mmsg, rmsg);
		mavlink_pub.publish(rmsg);
	}

	void mavlink_sub_cb(const Mavlink::ConstPtr &rmsg) {
		mavlink_message_t mmsg;

		if (mavutils::copy_ros_to_mavlink(rmsg, mmsg))
			UAS_FCU(&mav_uas)->send_message(&mmsg, rmsg->sysid, rmsg->compid);
		else
			ROS_ERROR("Drop mavlink packet: illegal payload64 size");
	}

	void plugin_route_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
		message_route_table[mmsg->msgid](mmsg, sysid, compid);
	}

	bool check_in_blacklist(std::string pl_name) {
		for (auto it = plugin_blacklist.cbegin();
				it != plugin_blacklist.cend();
				it++) {
			int cmp = fnmatch(it->c_str(), pl_name.c_str(), FNM_CASEFOLD);
			if (cmp == 0)
				return true;
			else if (cmp != FNM_NOMATCH)
				ROS_ERROR("Blacklist check error! fnmatch('%s', '%s')",
						it->c_str(), pl_name.c_str());
		}

		return false;
	}

	void add_plugin(std::string pl_name) {
		boost::shared_ptr<mavplugin::MavRosPlugin> plugin;

		if (check_in_blacklist(pl_name)) {
			ROS_INFO_STREAM("Plugin [alias " << pl_name << "] blacklisted");
			return;
		}

		try {
			plugin = plugin_loader.createInstance(pl_name);
			plugin->initialize(mav_uas, node_handle, diag_updater);
			loaded_plugins.push_back(plugin);
			std::string repr_name = plugin->get_name();

			ROS_INFO_STREAM("Plugin " << repr_name <<
					" [alias " << pl_name << "] loaded and initialized");

			auto sub_map = plugin->get_rx_handlers();
			for (auto it = sub_map.begin();
					it != sub_map.end();
					++it) {
				ROS_DEBUG("Route msgid %d to %s", it->first, repr_name.c_str());
				message_route_table[it->first].connect(it->second);
			}

		} catch (pluginlib::PluginlibException& ex) {
			ROS_ERROR_STREAM("Plugin load exception: " << ex.what());
		}
	}

	void terminate_cb() {
		ROS_ERROR("FCU connection closed, mavros will be terminated.");
		ros::requestShutdown();
	}

	void startup_px4_usb_quirk(void) {
		/* sample code from QGC */
		const uint8_t init[] = {0x0d, 0x0d, 0x0d, 0};
		const uint8_t nsh[] = "sh /etc/init.d/rc.usb\n";

		ROS_INFO("Autostarting mavlink via USB on PX4");
		UAS_FCU(&mav_uas)->send_bytes(init, 3);
		UAS_FCU(&mav_uas)->send_bytes(nsh, sizeof(nsh) - 1);
		UAS_FCU(&mav_uas)->send_bytes(init, 4);	/* NOTE in original init[3] */
	}

	void log_connect_change(bool connected) {
		/* note: sys_status plugin required */
		if (connected)
			ROS_INFO("CON: Got HEARTBEAT, connected.");
		else
			ROS_WARN("CON: Lost connection, HEARTBEAT timed out.");
	}
};


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mavros");
	ros::NodeHandle nh("~");

	MavRos mavros(nh);
	mavros.spin();

	return 0;
}


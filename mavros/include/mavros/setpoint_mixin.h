/**
 * @brief Mixin for setpoint plugins
 * @file setpoint_mixin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
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

#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>

#include <geometry_msgs/TransformStamped.h>

namespace mavplugin {
/**
 * @brief This mixin adds set_position_target_local_ned()
 *
 * @note derived class should provide UAS pointer in uas member.
 */
template <class D>
class SetPositionTargetLocalNEDMixin {
public:
	void set_position_target_local_ned(uint32_t time_boot_ms, uint8_t coordinate_frame,
			uint16_t type_mask,
			float x, float y, float z,
			float vx, float vy, float vz,
			float afx, float afy, float afz,
			float yaw, float yaw_rate) {
		UAS *_uas = static_cast<D *>(this)->uas;
		mavlink_message_t msg;
		mavlink_msg_set_position_target_local_ned_pack_chan(UAS_PACK_CHAN(_uas), &msg,
				time_boot_ms,	// why it not usec timestamp?
				UAS_PACK_TGT(_uas),
				coordinate_frame,
				type_mask,
				x, y, z,
				vx, vy, vz,
				afx, afy, afz,
				yaw, yaw_rate);
		UAS_FCU(_uas)->send_message(&msg);
	}
};

/**
 * @brief This mixin adds TF2 listener thread to plugin
 *
 * It requires tf_frame_id, tf_child_frame_id strings
 * tf_rate double and uas object pointer
 */
template <class D>
class TF2ListenerMixin {
public:
	std::thread tf_thread;
	std::string tf_thd_name;
	boost::function<void (const geometry_msgs::TransformStamped &)> tf_transform_cb;

	/**
	 * @brief start tf listener
	 *
	 * @param _thd_name  listener thread name
	 * @param cbp        plugin callback function
	 */
	void tf2_start(const char *_thd_name, void (D::*cbp)(const geometry_msgs::TransformStamped &) ) {
		tf_thd_name = _thd_name;
		tf_transform_cb = boost::bind(cbp, static_cast<D *>(this), _1);

		std::thread t(boost::bind(&TF2ListenerMixin::tf_listener, this));
		mavutils::set_thread_name(t, tf_thd_name);
		tf_thread.swap(t);
	}

	void tf_listener(void) {
		mavros::UAS *_uas = static_cast<D *>(this)->uas;
		std::string &_frame_id = static_cast<D *>(this)->tf_frame_id;
		std::string &_child_frame_id = static_cast<D *>(this)->tf_child_frame_id;

		ros::Rate rate(static_cast<D *>(this)->tf_rate);
		while (ros::ok()) {
			// Wait up to 3s for transform
			if (_uas->tf2_buffer.canTransform(_frame_id, _child_frame_id, ros::Time(0), ros::Duration(3.0))) {
				try {
					auto transform = _uas->tf2_buffer.lookupTransform(
							_frame_id, _child_frame_id, ros::Time(0), ros::Duration(3.0));
					tf_transform_cb(transform);
				}
				catch (tf2::LookupException &ex) {
					ROS_ERROR_NAMED("tf2_buffer", "%s: %s", tf_thd_name.c_str(), ex.what());
				}
			}
			rate.sleep();
		}
	}
};
};	// namespace mavplugin

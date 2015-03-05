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

#include <tf/transform_listener.h>

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
				time_boot_ms, // why it not usec timestamp?
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
 * @brief This mixin adds TF listener to plugin
 *
 * It requires ros::NodeHandle named sp_nh,
 * frame_id and child_frame_id strings and tf_rate double.
 */
template <class D>
class TFListenerMixin {
public:
	std::thread tf_thread;
	std::string thd_name;
	boost::function<void (const tf::Transform&, const ros::Time&)> tf_transform_cb;

	/**
	 * @brief start tf listener
	 *
	 * @param _thd_name  listener thread name
	 * @param cbp        plugin callback function
	 */
	void tf_start(const char *_thd_name, void (D::*cbp)(const tf::Transform&, const ros::Time&) ) {
		thd_name = _thd_name;
		tf_transform_cb = boost::bind(cbp, static_cast<D *>(this), _1, _2);

		std::thread t(boost::bind(&TFListenerMixin::tf_listener, this));
		mavutils::set_thread_name(t, thd_name);
		tf_thread.swap(t);
	}

	void tf_listener(void) {
		ros::NodeHandle &_sp_nh = static_cast<D *>(this)->sp_nh;
		std::string &_frame_id = static_cast<D *>(this)->frame_id;
		std::string &_child_frame_id = static_cast<D *>(this)->child_frame_id;

		tf::TransformListener listener(_sp_nh);
		tf::StampedTransform transform;
		ros::Rate rate(static_cast<D *>(this)->tf_rate);
		while (_sp_nh.ok()) {
			// Wait up to 3s for transform
			listener.waitForTransform(_frame_id, _child_frame_id, ros::Time(0), ros::Duration(3.0));
			try {
				listener.lookupTransform(_frame_id, _child_frame_id, ros::Time(0), transform);
				tf_transform_cb(static_cast<tf::Transform>(transform), transform.stamp_);
			}
			catch (tf::TransformException ex) {
				ROS_ERROR_NAMED("setpoint", "%s: %s", thd_name.c_str(), ex.what());
			}
			rate.sleep();
		}
	}
};

}; // namespace mavplugin

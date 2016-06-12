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

#include <functional>
#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>

#include <geometry_msgs/TransformStamped.h>

namespace mavros {
namespace plugin {
/**
 * @brief This mixin adds set_position_target_local_ned()
 */
template <class D>
class SetPositionTargetLocalNEDMixin {
public:
	void set_position_target_local_ned(uint32_t time_boot_ms, uint8_t coordinate_frame,
			uint16_t type_mask,
			Eigen::Vector3d p,
			Eigen::Vector3d v,
			Eigen::Vector3d af,
			float yaw, float yaw_rate)
	{
		mavros::UAS *m_uas_ = static_cast<D *>(this)->m_uas;
		mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED sp;

		sp.time_boot_ms = time_boot_ms;
		sp.coordinate_frame = coordinate_frame;
		sp.type_mask = type_mask;

		// [[[cog:
		// for fp, vp in (('', 'p'), ('v', 'v'), ('af', 'af')):
		//     for a in ('x', 'y', 'z'):
		//         cog.outl("sp.%s%s = %s.%s();" % (fp, a, vp, a))
		// ]]]
		sp.x = p.x();
		sp.y = p.y();
		sp.z = p.z();
		sp.vx = v.x();
		sp.vy = v.y();
		sp.vz = v.z();
		sp.afx = af.x();
		sp.afy = af.y();
		sp.afz = af.z();
		// [[[end]]] (checksum: f72768674b3c51e74aa1b4dd6d79b573)

		sp.yaw = yaw;
		sp.yaw_rate = yaw_rate;

		UAS_FCU(m_uas_)->send_message_ignore_drop(sp);
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

	/**
	 * @brief start tf listener
	 *
	 * @param _thd_name  listener thread name
	 * @param cbp        plugin callback function
	 */
	void tf2_start(const char *_thd_name, void (D::*cbp)(const geometry_msgs::TransformStamped &) )
	{
		tf_thd_name = _thd_name;
		auto tf_transform_cb = std::bind(cbp, static_cast<D *>(this), std::placeholders::_1);

		tf_thread = std::thread([this, tf_transform_cb]() {
			mavconn::utils::set_this_thread_name(tf_thd_name.c_str());

			mavros::UAS *m_uas_ = static_cast<D *>(this)->m_uas;
			std::string &_frame_id = static_cast<D *>(this)->tf_frame_id;
			std::string &_child_frame_id = static_cast<D *>(this)->tf_child_frame_id;

			ros::Rate rate(static_cast<D *>(this)->tf_rate);
			while (ros::ok()) {
				// Wait up to 3s for transform
				if (m_uas_->tf2_buffer.canTransform(_frame_id, _child_frame_id, ros::Time(0), ros::Duration(3.0))) {
					try {
						auto transform = m_uas_->tf2_buffer.lookupTransform(
								_frame_id, _child_frame_id, ros::Time(0), ros::Duration(3.0));
						tf_transform_cb(transform);
					}
					catch (tf2::LookupException &ex) {
						ROS_ERROR_NAMED("tf2_buffer", "%s: %s", tf_thd_name.c_str(), ex.what());
					}
				}
				rate.sleep();
			}
		});
	}
};
}	// namespace plugin
}	// namespace mavros

/**
 * @brief Dummy plugin
 * @file dummy.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
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

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
//#include <boost/variant.hpp>
#include <boost/any.hpp>

namespace mavplugin {

class Parameter {
public:
	//typedef boost::variant<uint8_t, int8_t, uint16_t, int16_t, float> param_t;
	typedef boost::any param_t;

	std::string param_id;
	param_t param_value;
	uint16_t param_index;
	uint16_t param_count;
	bool update_pending_ack;

	static param_t from_param_value(mavlink_param_value_t &pmsg) {
		mavlink_param_union_t uv;
		uv.param_float = pmsg.param_value;

		switch (pmsg.param_type) {
		case MAV_PARAM_TYPE_UINT8:
			return uv.param_uint8;
		case MAV_PARAM_TYPE_INT8:
			return uv.param_int8;
		case MAV_PARAM_TYPE_UINT16:
			return uv.param_uint16;
		case MAV_PARAM_TYPE_INT16:
			return uv.param_int16;
		case MAV_PARAM_TYPE_UINT32:
			return uv.param_uint32;
		case MAV_PARAM_TYPE_INT32:
			return uv.param_int32;
		case MAV_PARAM_TYPE_REAL32:
			return uv.param_float;

		default:
		case MAV_PARAM_TYPE_UINT64:
		case MAV_PARAM_TYPE_INT64:
		case MAV_PARAM_TYPE_REAL64:
			ROS_WARN_NAMED("mavros", "Unsupported param '%.16s' type: %d, index: %d of %d",
					pmsg.param_id, pmsg.param_type,
					pmsg.param_index, pmsg.param_count);
			return param_t();
		};
	}

	static param_t from_param_value_apm_quirk(mavlink_param_value_t &pmsg) {
		switch (pmsg.param_type) {
		case MAV_PARAM_TYPE_UINT8:
			return (uint8_t) pmsg.param_value;
		case MAV_PARAM_TYPE_INT8:
			return (int8_t) pmsg.param_value;
		case MAV_PARAM_TYPE_UINT16:
			return (uint16_t) pmsg.param_value;
		case MAV_PARAM_TYPE_INT16:
			return (int16_t) pmsg.param_value;
		case MAV_PARAM_TYPE_UINT32:
			return (uint32_t) pmsg.param_value;
		case MAV_PARAM_TYPE_INT32:
			return (int32_t) pmsg.param_value;
		case MAV_PARAM_TYPE_REAL32:
			return pmsg.param_value;

		default:
		case MAV_PARAM_TYPE_UINT64:
		case MAV_PARAM_TYPE_INT64:
		case MAV_PARAM_TYPE_REAL64:
			ROS_WARN_NAMED("mavros", "Unsupported param '%.16s' type: %d, index: %d of %d",
					pmsg.param_id, pmsg.param_type,
					pmsg.param_index, pmsg.param_count);
			return param_t();
		};
	}
};

class ParamPlugin : public MavRosPlugin {
public:
	ParamPlugin() {
	};

	void initialize(ros::NodeHandle &nh,
			const boost::shared_ptr<mavconn::MAVConnInterface> &mav_link_,
			diagnostic_updater::Updater &diag_updater,
			MavContext &context)
	{
		mav_link = mav_link_;
		mav_context = &context;

		//param_request_list();
	}

	std::string get_name() {
		return "Param";
	}

	std::vector<uint8_t> get_supported_messages() {
		return {
			MAVLINK_MSG_ID_PARAM_VALUE
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_param_value_t pmsg;
		mavlink_msg_param_value_decode(msg, &pmsg);

		std::string param_id(pmsg.param_id, sizeof(pmsg.param_id));

		boost::mutex::scoped_lock lock(lock_);
		// search
		std::map<std::string, Parameter>::iterator
			param_it = parameters.find(param_id);
		if (param_it != parameters.end()) {
			// parameter exists
			Parameter *p = &param_it->second;
			p->param_value = from_param_value(pmsg);
			p->update_pending_ack = false;

			ROS_WARN_STREAM_COND_NAMED((p->param_index != pmsg.param_index ||
					p->param_count != pmsg.param_count),
					"mavros",
					"Param " << param_id << " index/count changed! FCU changed?");
			ROS_DEBUG_STREAM_NAMED("mavros", "Update param " << param_id <<
					" (" << p->param_index << "/" << p->param_count <<
					")"); // value: " << p->param_value);
		}
		else {
			// insert new element
			Parameter p;
			p.param_id = param_id;
			p.param_index = pmsg.param_index;
			p.param_count = pmsg.param_count;
			p.param_value = from_param_value(pmsg);
			p.update_pending_ack = false;

			parameters[param_id] = p;

			ROS_DEBUG_STREAM_NAMED("mavros", "New param " << param_id <<
					" (" << p.param_index << "/" << p.param_count <<
					")"); // value: " << p.param_value);
		}
	}

private:
	std::map<std::string, Parameter> parameters;
	boost::mutex lock_;
	MavContext *mav_context;
	boost::shared_ptr<mavconn::MAVConnInterface> mav_link;

	inline Parameter::param_t from_param_value(mavlink_param_value_t &msg) {
		if (mav_context->is_ardupilotmega())
			return Parameter::from_param_value_apm_quirk(msg);
		else
			return Parameter::from_param_value(msg);
	}

	void param_request_list() {
		mavlink_message_t msg;
		mavlink_param_request_list_t rql = {
			mav_context->get_tgt_system(),
			mav_context->get_tgt_component()
		};

		mavlink_msg_param_request_list_encode(0, 0, &msg, &rql);
		mav_link->send_message(&msg);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ParamPlugin, mavplugin::MavRosPlugin)


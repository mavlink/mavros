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
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/any.hpp>

#include <mavros/ParamSet.h>
#include <mavros/ParamGet.h>
#include <mavros/ParamPull.h>
#include <mavros/ParamPush.h>

namespace mavplugin {

/**
 * @brief Parameter storage
 *
 * Stores parameter value.
 */
class Parameter {
public:
	typedef boost::any param_t;

	std::string param_id;
	param_t param_value;
	uint16_t param_index;
	uint16_t param_count;

	/**
	 * Convert mavlink_param_value_t to internal format
	 */
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

	/**
	 * Variation of @a Parameter::from_param_value with quirks for ArduPilotMega
	 */
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

	/**
	 * Convert internal type to std::string (for debugging)
	 */
	static std::string to_string_vt(param_t p) {
		std::ostringstream sout;

		if (p.type() == typeid(uint8_t))
			sout << (unsigned) boost::any_cast<uint8_t>(p) << " ubyte";
		else if (p.type() == typeid(int8_t))
			sout << (int) boost::any_cast<int8_t>(p) << " byte";
		else if (p.type() == typeid(uint16_t))
			sout << boost::any_cast<uint16_t>(p) << " ushort";
		else if (p.type() == typeid(int16_t))
			sout << boost::any_cast<int16_t>(p) << " short";
		else if (p.type() == typeid(uint32_t))
			sout << boost::any_cast<uint32_t>(p) << " uint";
		else if (p.type() == typeid(int32_t))
			sout << boost::any_cast<int32_t>(p) << " int";
		else if (p.type() == typeid(float))
			sout << boost::any_cast<float>(p) << " float";
		else {
			ROS_FATAL_STREAM_NAMED("mavros", "Wrong param_t type: " << p.type().name());
			sout << "UNK " << p.type().name();
		}

		return sout.str();
	};

	/**
	 * Convert internal type to mavlink_param_union_t
	 */
	static mavlink_param_union_t to_param_union(param_t p) {
		mavlink_param_union_t ret;

		if (p.type() == typeid(uint8_t)) {
			ret.param_uint8 = boost::any_cast<uint8_t>(p);
			ret.type = MAV_PARAM_TYPE_UINT8;
		}
		else if (p.type() == typeid(int8_t)) {
			ret.param_int8 = boost::any_cast<int8_t>(p);
			ret.type = MAV_PARAM_TYPE_INT8;
		}
		else if (p.type() == typeid(uint16_t)) {
			ret.param_uint16 = boost::any_cast<uint16_t>(p);
			ret.type = MAV_PARAM_TYPE_UINT16;
		}
		else if (p.type() == typeid(int16_t)){
			ret.param_int16 = boost::any_cast<int16_t>(p);
			ret.type = MAV_PARAM_TYPE_INT16;
		}
		else if (p.type() == typeid(uint32_t)) {
			ret.param_uint32 = boost::any_cast<uint32_t>(p);
			ret.type = MAV_PARAM_TYPE_UINT32;
		}
		else if (p.type() == typeid(int32_t)) {
			ret.param_int32 = boost::any_cast<int32_t>(p);
			ret.type = MAV_PARAM_TYPE_INT32;
		}
		else if (p.type() == typeid(float)) {
			ret.param_float = boost::any_cast<float>(p);
			ret.type = MAV_PARAM_TYPE_REAL32;
		}
		else {
			ROS_FATAL_STREAM_NAMED("mavros", "Wrong param_t type: " << p.type().name());
		}

		return ret;
	};

	/**
	 * Variation of @a Parameter::to_param_union with quirks for ArduPilotMega
	 */
	static mavlink_param_union_t to_param_union_apm_quirk(param_t p) {
		mavlink_param_union_t ret;

		if (p.type() == typeid(uint8_t)) {
			ret.param_float = boost::any_cast<uint8_t>(p);
			ret.type = MAV_PARAM_TYPE_UINT8;
		}
		else if (p.type() == typeid(int8_t)) {
			ret.param_float = boost::any_cast<int8_t>(p);
			ret.type = MAV_PARAM_TYPE_INT8;
		}
		else if (p.type() == typeid(uint16_t)) {
			ret.param_float = boost::any_cast<uint16_t>(p);
			ret.type = MAV_PARAM_TYPE_UINT16;
		}
		else if (p.type() == typeid(int16_t)){
			ret.param_float = boost::any_cast<int16_t>(p);
			ret.type = MAV_PARAM_TYPE_INT16;
		}
		else if (p.type() == typeid(uint32_t)) {
			ret.param_float = boost::any_cast<uint32_t>(p);
			ret.type = MAV_PARAM_TYPE_UINT32;
		}
		else if (p.type() == typeid(int32_t)) {
			ret.param_float = boost::any_cast<int32_t>(p);
			ret.type = MAV_PARAM_TYPE_INT32;
		}
		else if (p.type() == typeid(float)) {
			ret.param_float = boost::any_cast<float>(p);
			ret.type = MAV_PARAM_TYPE_REAL32;
		}
		else {
			ROS_FATAL_STREAM_NAMED("mavros", "Wrong param_t type: " << p.type().name());
		}

		return ret;
	};

	/**
	 * For get/set services
	 */
	static int64_t to_integer(param_t &p) {
		if (p.type() == typeid(uint8_t))
			return boost::any_cast<uint8_t>(p);
		else if (p.type() == typeid(int8_t))
			return boost::any_cast<int8_t>(p);
		else if (p.type() == typeid(uint16_t))
			return boost::any_cast<uint16_t>(p);
		else if (p.type() == typeid(int16_t))
			return boost::any_cast<int16_t>(p);
		else if (p.type() == typeid(uint32_t))
			return boost::any_cast<uint32_t>(p);
		else if (p.type() == typeid(int32_t))
			return boost::any_cast<int32_t>(p);
		else
			return 0;
	};

	static double to_real(param_t &p) {
		if (p.type() == typeid(float))
			return boost::any_cast<float>(p);
		else
			return 0.0;
	};
};


class ParamPlugin : public MavRosPlugin {
public:
	ParamPlugin() {
	};

	void initialize(ros::NodeHandle &nh,
			const boost::shared_ptr<mavconn::MAVConnInterface> &mav_link_,
			diagnostic_updater::Updater &diag_updater,
			MavContext &context,
			boost::asio::io_service &timer_service)
	{
		param_count = -1;
		mav_link = mav_link_;
		mav_context = &context;

		param_nh = ros::NodeHandle(nh, "param");

		pull_srv = param_nh.advertiseService("pull", &ParamPlugin::pull_cb, this);
		push_srv = param_nh.advertiseService("push", &ParamPlugin::push_cb, this);
		set_srv = param_nh.advertiseService("set", &ParamPlugin::set_cb, this);
		get_srv = param_nh.advertiseService("get", &ParamPlugin::get_cb, this);

		param_timer.reset(new boost::asio::deadline_timer(timer_service));
		mav_context->sig_connection_changed.connect(boost::bind(&ParamPlugin::connection_cb, this, _1));
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

		std::string param_id(pmsg.param_id,
				strnlen(pmsg.param_id, sizeof(pmsg.param_id)));

		boost::recursive_mutex::scoped_lock lock(mutex);
		// search
		std::map<std::string, Parameter>::iterator
			param_it = parameters.find(param_id);
		if (param_it != parameters.end()) {
			// parameter exists
			Parameter *p = &param_it->second;
			p->param_value = from_param_value(pmsg);

			ROS_WARN_STREAM_COND_NAMED((p->param_index != pmsg.param_index ||
					p->param_count != pmsg.param_count),
					"mavros",
					"Param " << param_id << " index/count changed! FCU changed?");
			ROS_DEBUG_STREAM_NAMED("mavros", "Update param " << param_id <<
					" (" << p->param_index << "/" << p->param_count <<
					") value: " << Parameter::to_string_vt(p->param_value));
		}
		else {
			// insert new element
			Parameter p;
			p.param_id = param_id;
			p.param_index = pmsg.param_index;
			p.param_count = pmsg.param_count;
			p.param_value = from_param_value(pmsg);

			parameters[param_id] = p;

			ROS_DEBUG_STREAM_NAMED("mavros", "New param " << param_id <<
					" (" << p.param_index << "/" << p.param_count <<
					") value: " << Parameter::to_string_vt(p.param_value));
		}

		if (in_list_receiving) {
			/* we received first param. setup list timeout */
			if (param_count == -1) {
				param_count = pmsg.param_count;
				param_timer->cancel();
				param_timer->expires_from_now(boost::posix_time::milliseconds(LIST_TIMEOUT_MS));
				param_timer->async_wait(boost::bind(&ParamPlugin::restart_fetch_cb, this, _1));
			}

			/* index starting from 0, receivig done */
			if (pmsg.param_index + 1 == param_count) {
				in_list_receiving = false;
				param_timer->cancel();
				list_receiving.notify_all();
			}
		}
	}

private:
	boost::recursive_mutex mutex;
	MavContext *mav_context;
	boost::shared_ptr<mavconn::MAVConnInterface> mav_link;

	ros::NodeHandle param_nh;
	ros::ServiceServer pull_srv;
	ros::ServiceServer push_srv;
	ros::ServiceServer set_srv;
	ros::ServiceServer get_srv;

	std::unique_ptr<boost::asio::deadline_timer> param_timer;
	const int BOOTUP_TIME_MS = 10000;	//!< APM boot time
	const int PARAM_TIMEOUT_MS = 1000;	//!< Param wait time
	const int LIST_TIMEOUT_MS = 30000;	//!< Receive all time

	std::map<std::string, Parameter> parameters;
	ssize_t param_count;
	bool in_list_receiving;
	boost::condition_variable list_receiving;

	inline Parameter::param_t from_param_value(mavlink_param_value_t &msg) {
		if (mav_context->is_ardupilotmega())
			return Parameter::from_param_value_apm_quirk(msg);
		else
			return Parameter::from_param_value(msg);
	}

	inline mavlink_param_union_t to_param_union(Parameter::param_t p) {
		if (mav_context->is_ardupilotmega())
			return Parameter::to_param_union_apm_quirk(p);
		else
			return Parameter::to_param_union(p);
	}

	void param_request_list() {
		mavlink_message_t msg;

		mavlink_msg_param_request_list_pack(0, 0, &msg,
				mav_context->get_tgt_system(),
				mav_context->get_tgt_component()
				);
		mav_link->send_message(&msg);
	}

	void param_request_read(std::string id, int16_t index=-1) {
		ROS_ASSERT(index >= -1);

		mavlink_message_t msg;
		char param_id[sizeof(mavlink_param_request_read_t::param_id)];

		if (index != -1) {
			// by specs if len < 16: place null termination
			// else if len == 16: don't
			strncpy(param_id, id.c_str(), sizeof(param_id));
		}
		else
			param_id[0] = '\0'; // force NULL termination

		mavlink_msg_param_request_read_pack(0, 0, &msg,
				mav_context->get_tgt_system(),
				mav_context->get_tgt_component(),
				param_id,
				index
				);
		mav_link->send_message(&msg);
	}

	void param_set(Parameter &param) {
		mavlink_param_union_t pu = to_param_union(param.param_value);

		mavlink_message_t msg;
		char param_id[sizeof(mavlink_param_set_t::param_id)];
		strncpy(param_id, param.param_id.c_str(), sizeof(param_id));

		mavlink_msg_param_set_pack(0, 0, &msg,
				mav_context->get_tgt_system(),
				mav_context->get_tgt_component(),
				param_id,
				pu.param_float,
				pu.type
				);
		mav_link->send_message(&msg);
	}

	void connection_cb(bool connected) {
		boost::recursive_mutex::scoped_lock lock(mutex);
		if (connected) {
			parameters.clear();
			param_count = -1;

			param_timer->cancel();
			param_timer->expires_from_now(boost::posix_time::milliseconds(BOOTUP_TIME_MS));
			param_timer->async_wait(boost::bind(&ParamPlugin::start_fetch_cb, this, _1));
		}
		else {
			/* TODO: stop param requests */
			param_timer->cancel();
		}
	}

	void start_fetch_cb(boost::system::error_code error) {
		if (error)
			return;

		boost::recursive_mutex::scoped_lock lock(mutex);

		ROS_DEBUG_NAMED("mavros", "Requesting parameters");
		in_list_receiving = true;
		param_request_list();

		// resend PARAM_REQUEST_LIST if no PARAM_VALUE received before timeout
		param_timer->expires_from_now(boost::posix_time::milliseconds(PARAM_TIMEOUT_MS));
		param_timer->async_wait(boost::bind(&ParamPlugin::start_fetch_cb, this, _1));
	}

	void restart_fetch_cb(boost::system::error_code error) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		in_list_receiving = false;
		if (!error) {
			ROS_WARN_NAMED("mavros", "Request parameter list timed out!");
			param_count = -1;
		}

		start_fetch_cb(error);
	}

	bool wait_fetch_all() {
		boost::mutex cond_mutex;
		boost::unique_lock<boost::mutex> lock(cond_mutex);

		return list_receiving.timed_wait(lock, boost::posix_time::milliseconds(LIST_TIMEOUT_MS));
	}

	bool fetch_all() {
		start_fetch_cb(boost::system::error_code());
		return wait_fetch_all();
	}

	/**
	 * @brief fetches all parameters from device
	 * @service ~param/pull
	 */
	bool pull_cb(mavros::ParamPull::Request &req,
			mavros::ParamPull::Response &res) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (!in_list_receiving && param_count < 0) {
			lock.unlock();
			res.success = fetch_all();
		}
		else if (in_list_receiving) {
			lock.unlock();
			res.success = wait_fetch_all();
		}
		else {
			lock.unlock();
			res.success = true;
		}

		lock.lock();
		res.param_received = parameters.size();

		/* TODO: prameters -> rosparam */

		return true;
	}

	/**
	 * @brief push all parameter value to device
	 * @service ~param/push
	 * TODO blocking, success return
	 */
	bool push_cb(mavros::ParamPush::Request &req,
			mavros::ParamPush::Response &res) {
		return false;
	}

	/**
	 * @brief sets parameter value
	 * @service ~param/set
	 */
	bool set_cb(mavros::ParamSet::Request &req,
			mavros::ParamSet::Response &res) {
		return false;
	}

	/**
	 * @brief get parameter
	 * @service ~param/get
	 */
	bool get_cb(mavros::ParamGet::Request &req,
			mavros::ParamGet::Response &res) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		std::map<std::string, Parameter>::iterator
			param_it = parameters.find(req.param_id);
		if (param_it != parameters.end()) {
			Parameter *p = &param_it->second;

			res.success = true;
			res.integer = Parameter::to_integer(p->param_value);
			res.real = Parameter::to_real(p->param_value);
		}
		else {
			ROS_WARN_STREAM_NAMED("mavros", "ParamGet: Unknown parameter: " << req.param_id);
			res.success = false;
		}

		return true;
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ParamPlugin, mavplugin::MavRosPlugin)


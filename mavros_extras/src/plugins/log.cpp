#include <iostream>
#include <fstream>
#include <set>
#include <mutex>
#include <string>

#include <mavros/mavros_plugin.h>
#include <std_srvs/Trigger.h>

namespace mavros {
namespace extra_plugins {
using lock_guard = std::lock_guard<std::mutex>;
using utils::enum_value;
using mavlink::ardupilotmega::msg::REMOTE_LOG_BLOCK_STATUS;
using mavlink::ardupilotmega::msg::REMOTE_LOG_DATA_BLOCK;
using mavlink::ardupilotmega::MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS;
using mavlink::ardupilotmega::MAV_REMOTE_LOG_DATA_BLOCK_STATUSES;

class LogPlugin : public plugin::PluginBase {

public:
	LogPlugin() : PluginBase(),
		nh("~mavlink_logging"),
        stopped(false)
    {}

	void initialize(UAS& uas) override
	{
		PluginBase::initialize(uas);

        _last_block_s = ros::Time::now().toSec();

        nh.param<bool>("start_on_conn", start_log_on_init, true);
        nh.param<std::string>("path", path, "");

        start_srv = nh.advertiseService("start",
					&LogPlugin::start_cb, this);
        stop_srv = nh.advertiseService("stop",
					&LogPlugin::stop_cb, this);

        rotate_log_timer = nh.createTimer(ros::Duration(1.),
                    &LogPlugin::rotate_on_disarm, this);
        rotate_log_timer.stop();

        send_nacks_timer = nh.createTimer(ros::Duration(0.1),
                    &LogPlugin::send_nacks_for_missing_blocks, this);
        send_nacks_timer.stop();

        enable_connection_cb();
    }

    Subscriptions get_subscriptions() override
	{
		return {
			       make_handler(&LogPlugin::handle_data),
		};
	}

private:
    ros::NodeHandle nh;
    
    ros::ServiceServer start_srv;
    ros::ServiceServer stop_srv;

    ros::Timer rotate_log_timer;
	ros::Timer send_nacks_timer;
    
    bool start_log_on_init;
    std::string path;
    bool armed;
    bool stopped;
    double _last_block_s;
    std::mutex mutex;
    std::set<uint32_t> blocks_received;
    std::ofstream log;

    //----- Services -----//

    bool start_cb(std_srvs::Trigger::Request &req,
				std_srvs::Trigger::Response &res)
    {
        if (_send_logging_start() && _open_logfile())
        {
            stopped = false;
            rotate_log_timer.start();
            send_nacks_timer.start();
		    res.success = true;
            ROS_INFO_NAMED("LG", "LG: Starting logging via Mavlink");
		} else {
            res.success = false;
            ROS_INFO_NAMED("LG", "LG: Failed to start logging via Mavlink");
		}
		return true;
    }

    bool stop_cb(std_srvs::Trigger::Request &req,
				std_srvs::Trigger::Response &res)
    {
        if (_send_logging_stop() && _close_logfile())
        {
            stopped = true;
            rotate_log_timer.stop();
            send_nacks_timer.stop();
		    res.success = true;
            ROS_INFO_NAMED("LG", "LG: Stopping logging via Mavlink");
		} else {
			res.success = false;
            ROS_INFO_NAMED("LG", "LG: Failed to stop logging via Mavlink");
		}
		return true;
    }

    //----- Callbacks -----//

    // Act on first heartbeat from FCU
	void connection_cb(bool connected) override
	{
		if (connected && start_log_on_init && m_uas->is_ardupilotmega()) {
            armed = m_uas->get_armed();
            _close_logfile();
            _send_logging_stop();
            _open_logfile();
            _send_logging_start();
            stopped = false;
            rotate_log_timer.start();
            send_nacks_timer.start();
		}
	}

    void handle_data(const mavlink::mavlink_message_t *msg, REMOTE_LOG_DATA_BLOCK &lmsg)
    {
        _last_block_s = ros::Time::now().toSec();
        if (stopped)
        { // still reciving log data when requested to stop 
            _send_logging_stop();
        } else {
            if (_log_is_open()){ // check the file is open now
                _write_to_file(lmsg);
                _send_ack(lmsg.seqno);
                ROS_DEBUG_NAMED("LG", "LG: Received data: seqno: %d pckt size: %lu", lmsg.seqno, lmsg.data.size());
            } 
            else {
              //if file cannot be open, dont send an ACK, we'll try again later
                ROS_DEBUG_NAMED("LG", "LG: No logfile open but recv: seqno: %d", lmsg.seqno);
            }
        }
    }

    //----- Timers -----//

    void rotate_on_disarm(const ros::TimerEvent &event)
    {  //on disarm, stop the old log, and start a new one
        if (m_uas->get_armed() != armed) {
            armed = m_uas->get_armed();
            if (!armed) {
                ROS_INFO_NAMED("LG", "LG: Rotating logfile on disarm");
                
                double _start_s = ros::Time::now().toSec();
                bool _stoppped_receiving = false;
                bool _timedout = false;
                while (!_stoppped_receiving && !_timedout) {
                    double _now_s = ros::Time::now().toSec();
                    _stoppped_receiving = (_now_s - _last_block_s) > 3;  //wait 3s before closing file
                    _timedout = (_now_s - _start_s) > 30;  //timeout after 30s waiting for stop
                }

                for (int i = 0; i < 3; ++i) {  //send three times for sender gets the message
                  // mavproxy does this, so we will copy
                    _send_logging_stop();
                }

                _close_logfile(); //close the logfile
                _open_logfile(); //open a new logfiles
                _send_logging_start(); //start a new logfile
            }
        }
    }

    void send_nacks_for_missing_blocks(const ros::TimerEvent &event)
    {  //iiterate through the set of recived block, send a nack for missing ones
        lock_guard lock(mutex);  //hold the mutex for the block_recieved set
        if(!blocks_received.empty())
        {
            for (uint32_t expected = 0; expected < *blocks_received.rbegin(); ++expected)
            {
                if (blocks_received.find(expected) == blocks_received.end()) {
                    _send_nack(expected);
                }
            }
        }
    }

    //----- Helpers -----//

    bool _log_is_open(void)
    {
        return log.is_open();
    }

    bool _open_logfile(void)
    {  // Open a new logfile to write to
        int now = static_cast<int>(ros::Time::now().toSec());
        log.open(path + std::to_string(now) + ".bin", std::ios::out | std::ios::binary);
        return log.is_open();
    }

    bool _close_logfile(void)
    {  // Close the logfile
        log.close();
        lock_guard lock(mutex);  //hold the mutex for the block_recieved set, released when out scope
        blocks_received.clear();  //clear the set
        return !log.is_open();
    }

    void _write_to_file(REMOTE_LOG_DATA_BLOCK &lmsg)
    {  //write the log data to the file
        log.seekp(lmsg.seqno*lmsg.data.size(), std::ios::beg);
        log.write((const char*)&lmsg.data[0], lmsg.data.size());
        lock_guard lock(mutex);  //hold the mutex for the block_recieved set, released when out scope
        blocks_received.insert(lmsg.seqno);  //insert block into the recived set
    }

    bool _send_ack(uint32_t seqno)
    {  // Send ACK in response to a receieved log data block
        REMOTE_LOG_BLOCK_STATUS smsg = {};
		m_uas->msg_set_target(smsg);
		smsg.seqno = seqno;
        smsg.status = enum_value(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES::ACK);
        UAS_FCU(m_uas)->send_message_ignore_drop(smsg);
        ROS_DEBUG_NAMED("LG", "LG: Sending ACK: seqno: %d", seqno);
    }

    bool _send_nack(uint32_t seqno)
    {  // Send a NACK for missing log data blocks
        REMOTE_LOG_BLOCK_STATUS smsg = {};
		m_uas->msg_set_target(smsg);
		smsg.seqno = seqno;
        smsg.status = enum_value(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES::NACK);
        UAS_FCU(m_uas)->send_message_ignore_drop(smsg);
        ROS_DEBUG_NAMED("LG", "LG: Sending NACK: seqno: %d", seqno);
    }

    bool _send_logging_start(void)
    {  // Send command to start the logging via mavlink
        REMOTE_LOG_BLOCK_STATUS msg = {};
		m_uas->msg_set_target(msg);
		msg.seqno = enum_value(MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS::START);
        msg.status = enum_value(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES::ACK);
        try {
			UAS_FCU(m_uas)->send_message(msg);
            return true;
		} catch (std::length_error&) {
			return false;
		} 
    }

    bool _send_logging_stop(void)
    {  // Send command to stop the logging via mavlink
        REMOTE_LOG_BLOCK_STATUS msg = {};
		m_uas->msg_set_target(msg);
		msg.seqno = enum_value(MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS::STOP);
        msg.status = enum_value(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES::ACK);
        try {
			UAS_FCU(m_uas)->send_message(msg);
            return true;
		} catch (std::length_error&) {
			return false;
		} 
    }
};

}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::LogPlugin, mavros::plugin::PluginBase)
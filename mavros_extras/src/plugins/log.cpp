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
        start_log_on_init(false),
        path(""),
        stopped(true),
        armed(false),
        _last_block_s(0.0)
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

        //check_pre_arm_timer = nh.createTimer(ros::Duration(1.),
        //            &LogPlugin::check_for_logging_pre_arm, this);
        //check_pre_arm_timer.stop();

        rotate_log_timer = nh.createTimer(ros::Duration(0.1),
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

    //ros::Timer check_pre_arm_timer;
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
        if (send_logging_start() && open_logfile())
        {
            stopped = false;
            start_timers();
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
        if (send_logging_stop() && close_logfile())
        {
            stopped = true;
            stop_timers();
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
            //close_logfile();
            //send_logging_stop();
            //open_logfile();
            //send_logging_start();
            stopped = false;
            start_timers();
            ROS_INFO_NAMED("LG", "LG: Starting logging via Mavlink (on connection)");
        }
    }

    void handle_data(const mavlink::mavlink_message_t *msg, REMOTE_LOG_DATA_BLOCK &lmsg)
    {
        _last_block_s = ros::Time::now().toSec();
        if (stopped)
        { // still reciving log data when requested to stop
            send_logging_stop();
        } else {

            if (lmsg.seqno == 0 && !log_is_open()) {  //check if logfile needs to be opened
                open_logfile();
            }

            if (log_is_open()){ // check the file is open now
                write_to_file(lmsg);
                send_ack(lmsg.seqno);
                ROS_DEBUG_NAMED("LG", "LG: Received data: seqno: %d pckt size: %lu", lmsg.seqno, lmsg.data.size());
            }
            else {
              //if file cannot be open, dont send an ACK, we'll try again later
                ROS_DEBUG_NAMED("LG", "LG: No logfile open but recv: seqno: %d", lmsg.seqno);
            }
        }
    }

    //----- Timers -----//

    /* check if need to send logging start because of pre-arm
    void check_for_logging_pre_arm(const ros::TimerEvent &event)
    {  //this function should only be needed after a soft-reboot
       using STS = mavlink::common::MAV_SYS_STATUS_SENSOR;
       if (!stopped && log_is_open())
       {
           if (m_uas->get_onboard_control_sensors_enabled() & enum_value(STS::LOGGING))
           {  //check if logging is eneabled
               if (m_uas->get_onboard_control_sensors_health() & enum_value(STS::LOGGING))
               {  //if logging has failed, send start
                   send_logging_start();
               }
           }
       }
    } */

    void rotate_on_disarm(const ros::TimerEvent &event)
    {  //on disarm, stop the old log, and start a new one
        if (!stopped)
        {
            if (m_uas->get_armed() != armed) 
            {
                armed = m_uas->get_armed();
                if (!armed) 
                {
                    //ROS_INFO_NAMED("LG", "LG: Rotating logfile on disarm");

                    /* double _start_s = ros::Time::now().toSec();
                    bool _stoppped_receiving = false;
                    bool _timedout = false;
                    while (!_stoppped_receiving && !_timedout) {
                        double _now_s = ros::Time::now().toSec();
                        _stoppped_receiving = (_now_s - _last_block_s) > 3;  //wait 3s before closing file
                        _timedout = (_now_s - _start_s) > 30;  //timeout after 30s waiting for stop
                    } */

                    ROS_INFO_NAMED("LG", "LG: Closing logfile on disarm");

                    for (int i = 0; i < 3; ++i) {  //send three times for sender gets the message
                    // mavproxy does this, so we will copy
                        send_logging_stop();
                    }

                    close_logfile(); //close the logfile
                    //open_logfile(); //open a new logfiles
                    //send_logging_start(); //start a new logfile
                } else {
                    ROS_INFO_NAMED("LG", "LG: Opening logfile on arm");

                    open_logfile();
                    send_logging_start();
                } 
            }
        }
    }

    void send_nacks_for_missing_blocks(const ros::TimerEvent &event)
    {  //iterate through the set of recived block, send a nack for missing ones
        if (!stopped)
        {
            lock_guard lock(mutex);  //hold the mutex for the block_recieved set
            if(!blocks_received.empty())
            {
                for (uint32_t expected = 0; expected < *blocks_received.rbegin(); ++expected)
                {
                    if (blocks_received.find(expected) == blocks_received.end()) {
                        send_nack(expected);
                    }
                }
            }
        }
    }

    //----- Helpers -----//

    void stop_timers(void)
    {
        //check_pre_arm_timer.stop();
        rotate_log_timer.stop();
        send_nacks_timer.stop();
    }

    void start_timers(void)
    {
        //check_pre_arm_timer.start();
        rotate_log_timer.start();
        send_nacks_timer.start(); 
    }

    bool log_is_open(void)
    {
        return log.is_open();
    }

    bool open_logfile(void)
    {  // Open a new logfile to write to
        int now = static_cast<int>(ros::Time::now().toSec());
        log.open(path + std::to_string(now) + ".bin", std::ios::out | std::ios::binary);
        return log.is_open();
    }

    bool close_logfile(void)
    {  // Close the logfile
        log.close();
        lock_guard lock(mutex);  //hold the mutex for the block_recieved set, released when out scope
        blocks_received.clear();  //clear the set
        return !log.is_open();
    }

    void write_to_file(REMOTE_LOG_DATA_BLOCK &lmsg)
    {  //write the log data to the file
        log.seekp(lmsg.seqno*lmsg.data.size(), std::ios::beg);
        log.write((const char*)&lmsg.data[0], lmsg.data.size());
        lock_guard lock(mutex);  //hold the mutex for the block_recieved set, released when out scope
        blocks_received.insert(lmsg.seqno);  //insert block into the recived set
    }

    bool send_ack(uint32_t seqno)
    {  // Send ACK in response to a receieved log data block
        REMOTE_LOG_BLOCK_STATUS smsg = {};
        m_uas->msg_set_target(smsg);
        smsg.seqno = seqno;
        smsg.status = enum_value(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES::ACK);
        UAS_FCU(m_uas)->send_message_ignore_drop(smsg);
        ROS_DEBUG_NAMED("LG", "LG: Sending ACK: seqno: %d", seqno);
    }

    bool send_nack(uint32_t seqno)
    {  // Send a NACK for missing log data blocks
        REMOTE_LOG_BLOCK_STATUS smsg = {};
        m_uas->msg_set_target(smsg);
        smsg.seqno = seqno;
        smsg.status = enum_value(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES::NACK);
        UAS_FCU(m_uas)->send_message_ignore_drop(smsg);
        ROS_DEBUG_NAMED("LG", "LG: Sending NACK: seqno: %d", seqno);
    }

    bool send_logging_start(void)
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

    bool send_logging_stop(void)
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
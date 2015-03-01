/**
 * @brief SetpointActuatorControl plugin
 * @file setpoint_actuator_control.cpp
 * @author Marcel Stüttgen <stuettgen@fh-aachen.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Marcel Stüttgen.
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

#include<std_msgs/Float32MultiArray.h>


namespace mavplugin {

/**
 * @brief Setpoint actuator control plugin
 *
 * Send setpoint actuator control to FCU controller.
 */
class ActuatorControlPlugin : public MavRosPlugin {

 public:

  //constructor
  ActuatorControlPlugin() : uas(nullptr) { };

  //init function
  void initialize(UAS &uas_, ros::NodeHandle &nh, diagnostic_updater::Updater &diag_updater) {

    uas = &uas_;
    sp_nh_ = ros::NodeHandle(nh, "setpoint_actuator_control");
    controls_sub_ = sp_nh_.subscribe("actuator_controls", 10, &ActuatorControlPlugin::control_cb, this);

  }

  //name of object
  const std::string get_name() const {
    return "SetpointActuatorControl";
  }

  const message_map get_rx_handlers() {
    return { /* Rx disabled */ };
  }

private:
 
  UAS *uas;

  ros::NodeHandle sp_nh_;
  ros::Subscriber controls_sub_;

  /* -*- low-level send -*- */
  /* message definiton here: https://pixhawk.ethz.ch/mavlink/#SET_ACTUATOR_CONTROL_TARGET */

  void set_actuator_control_target(uint32_t time_boot_ms, uint8_t group_mix,float controls[8]) {

    mavlink_message_t msg;
    //todo: get correckt pack chan msg
    /*
    mavlink_msg_set_actuator_control_target_pack_chan(UAS_PACK_CHAN(uas), &msg, 
                                                      time_boot_ms,
                                                      UAS_PACK_TGT(uas),
                                                      group_mix,
                                                      controls)
    */
    UAS_FCU(uas)->send_message(&msg);
  }

  
  /* -*- callbacks -*- */
  
  void control_cb(const std_msgs::Float32MultiArray::ConstPtr &req) {
    uint8_t group_mix = 0; //todo: get right group mix
    
    float controls[8];
    /* 0: roll
       1: pitch
       2: yaw
       3: throttle
       4: flaps
       5: spoilers
       6: airbrakes
       7: landing gear*/

    //copy values
    for (int i = 0; i < 8; i++) {
      controls[i]=req->data[i];
    }

    //call low level send
    set_actuator_control_target(ros::Time::now().toNSec()/1000000,
                                group_mix,
                                controls);                                
  }

};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ActuatorControlPlugin, mavplugin::MavRosPlugin)

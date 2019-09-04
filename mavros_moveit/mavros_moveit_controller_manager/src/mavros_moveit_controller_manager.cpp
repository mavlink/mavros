/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Saifullah */

#include <algorithm>
#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <mavros_moveit_controller_manager/follow_multi_dof_joint_trajectory_controller_handle.h>
#include <moveit/utils/xmlrpc_casts.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <map>

using namespace moveit::core;
using namespace moveit_simple_controller_manager;

const std::string LOGNAME("MavrosMoveItControllerManager");

namespace mavros_moveit_controller_manager
{
class MavrosMoveItControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  MavrosMoveItControllerManager() : node_handle_("~")
  {
    ROS_INFO("Initializing MavrosMoveItControllerManager...");

    if (!node_handle_.hasParam("controller_list"))
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "No controller_list specified.");
      return;
    }

    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);
    if (!isArray(controller_list))
    {
      ROS_ERROR_NAMED(LOGNAME, "Parameter controller_list should be specified as an array");
      return;
    }

    /* actually create each controller */
    for (int i = 0; i < controller_list.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      ROS_INFO_STREAM("Initializing controller:" << i);
      if (!isStruct(controller_list[i], { "name", "joints", "action_ns", "type" }))
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "name, joints, action_ns, and type must be specifed for each controller");
        continue;
      }

      try
      {
        const std::string name = std::string(controller_list[i]["name"]);
        const std::string action_ns = std::string(controller_list[i]["action_ns"]);
        const std::string type = std::string(controller_list[i]["type"]);

        if (!isArray(controller_list[i]["joints"]))
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "The list of joints for controller " << name
                                                                               << " is not specified as an array");
          continue;
        }

        ROS_INFO_STREAM("name:" << name);
        ROS_INFO_STREAM("type:" << type);
        ROS_INFO_STREAM("action_ns:" << action_ns);
        ActionBasedControllerHandleBasePtr new_handle;
        if(type == "FollowMultiDofJointTrajectory")
        {
          using namespace mavros_moveit_controller_manager;
          new_handle.reset(new FollowMultiDofJointTrajectoryControllerHandle(name, action_ns));
          if (static_cast<FollowMultiDofJointTrajectoryControllerHandle*>(new_handle.get())->isConnected())
          {
            ROS_INFO_STREAM("MavrosMoveItControllerManager: Added FollowMultiDofJointTrajectory controller for " << name );
            controllers_[name] = new_handle;
          }
        }
        else
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Unknown controller type: " << type.c_str());
          continue;
        }
        if (!controllers_[name])
        {
          controllers_.erase(name);
          continue;
        }

        /* add list of joints, used by controller manager and MoveIt */
        for (int j = 0; j < controller_list[i]["joints"].size(); ++j)
          new_handle->addJoint(std::string(controller_list[i]["joints"][j]));

        new_handle->configure(controller_list[i]);
      }
      catch (...)
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Caught unknown exception while parsing controller information");
      }
    }
  }

  ~MavrosMoveItControllerManager() override = default;

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return static_cast<moveit_controller_manager::MoveItControllerHandlePtr>(it->second);
    else
      ROS_FATAL_STREAM_NAMED(LOGNAME, "No such controller: " << name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  void getControllersList(std::vector<std::string>& names) override
  {
    for (std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.begin();
         it != controllers_.end(); ++it)
      names.push_back(it->first);
    ROS_INFO_STREAM_NAMED(LOGNAME, "Returned " << names.size() << " controllers in list");
  }

  /*
   * This plugin assumes that all controllers are already active -- and if they are not, well, it has no way to deal
   * with it anyways!
   */
  void getActiveControllers(std::vector<std::string>& names) override
  {
    getControllersList(names);
  }

  /*
   * Controller must be loaded to be active, see comment above about active controllers...
   */
  virtual void getLoadedControllers(std::vector<std::string>& names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  void getControllerJoints(const std::string& name, std::vector<std::string>& joints) override
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      it->second->getJoints(joints);
    }
    else
    {
      ROS_WARN_NAMED(LOGNAME, "The joints for controller '%s' are not known. Perhaps the controller configuration is "
                              "not loaded on the param server?",
                     name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all active and default -- that's what makes this thing simple.
   */
  moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string& name) override
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /* Cannot switch our controllers */
  bool switchControllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate) override
  {
    return false;
  }

protected:
  ros::NodeHandle node_handle_;
  std::map<std::string, ActionBasedControllerHandleBasePtr> controllers_;
};

}  // end namespace moveit_simple_controller_manager

PLUGINLIB_EXPORT_CLASS(mavros_moveit_controller_manager::MavrosMoveItControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
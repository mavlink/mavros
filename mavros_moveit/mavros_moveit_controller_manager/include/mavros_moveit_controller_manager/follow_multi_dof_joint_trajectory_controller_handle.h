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

#ifndef MOVEIT_PLUGINS_FOLLOW_MULTI_DOF_JOINT_TRAJECTORY_CONTROLLER_HANDLE
#define MOVEIT_PLUGINS_FOLLOW_MULTI_DOF_JOINT_CONTROLLER_HANDLE

#include <control_msgs/JointTolerance.h>
#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <mavros_moveit_actions/FollowMultiDofJointTrajectoryAction.h>

namespace mavros_moveit_controller_manager
{
class FollowMultiDofJointTrajectoryControllerHandle
    : public moveit_simple_controller_manager::ActionBasedControllerHandle<mavros_moveit_actions::FollowMultiDofJointTrajectoryAction>
{
public:
  FollowMultiDofJointTrajectoryControllerHandle(const std::string& name, const std::string& action_ns)
    : ActionBasedControllerHandle<mavros_moveit_actions::FollowMultiDofJointTrajectoryAction>(name, action_ns)
  {
  }

  bool sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory) override;

  void configure(XmlRpc::XmlRpcValue& config) override;

protected:
  void configure(XmlRpc::XmlRpcValue& config, const std::string& config_name,
                 std::vector<control_msgs::JointTolerance>& tolerances);
  static control_msgs::JointTolerance& getTolerance(std::vector<control_msgs::JointTolerance>& tolerances,
                                                    const std::string& name);

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const mavros_moveit_actions::FollowMultiDofJointTrajectoryResultConstPtr& result);

  void controllerActiveCallback();

  void controllerFeedbackCallback(const mavros_moveit_actions::FollowMultiDofJointTrajectoryFeedbackConstPtr& feedback);

  mavros_moveit_actions::FollowMultiDofJointTrajectoryGoal goal_template_;
};

}  // end namespace mavros_moveit_controller_manager

#endif  // MOVEIT_PLUGINS_EXECUTE_DRONE_TRAJECTORY_CONTROLLER_HANDLE
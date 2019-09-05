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

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_moveit_actions/FollowMultiDofJointTrajectoryAction.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <string>

class FollowMultiDofJointTrajectoryAction
{
    typedef actionlib::SimpleActionServer<mavros_moveit_actions::FollowMultiDofJointTrajectoryAction> ActionServer;
    typedef mavros_moveit_actions::FollowMultiDofJointTrajectoryResult Result;
    typedef mavros_moveit_actions::FollowMultiDofJointTrajectoryFeedback Feedback;
    typedef mavros_moveit_actions::FollowMultiDofJointTrajectoryGoalConstPtr GoalPtr;
public:
    FollowMultiDofJointTrajectoryAction(const std::string& name) : 
        action_name_(name), 
        action_server_(nh_, name, boost::bind(&FollowMultiDofJointTrajectoryAction::executeCb, this, _1), false)
    {
        // setup publishers/subscribers/services
        state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &FollowMultiDofJointTrajectoryAction::stateCb, this);
        local_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &FollowMultiDofJointTrajectoryAction::poseCb, this);
        local_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // start the action server
        action_server_.start();
    }

    void idle() {
        while(ros::ok()){
            if (!executing)
                rate_ = ros::Rate(4);
            ros::spinOnce();
            rate_.sleep();
        }
    }

    void executeCb(const GoalPtr &goal) {
        auto success = true;

        // check for FCU connection
        if (current_state_.connected) {
            // trajectory execution started
            executing = true;

            // setpoint publishing rate MUST be faster than 2Hz. From mavros documentation
            rate_ = ros::Rate(20.0);

            current_pose_.pose.position.z = 10.0;
            //send a few setpoints before starting
            for(int i = 10; ros::ok() && i > 0; --i) {
                local_pose_pub_.publish(current_pose_);
                ros::spinOnce();
                rate_.sleep();
            }

            if (!current_state_.armed) {
                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = true;
                if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle successfully armed.");
                } else {
                    ROS_WARN("Vehicle could not be enabled. Cannot execute moveit trajectory.");
                    return;
                }
            }

            if (current_state_.mode != "OFFBOARD") {
                mavros_msgs::SetMode offb_set_mode;
                offb_set_mode.request.custom_mode = "OFFBOARD";
                if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("Mode OFFBOARD enabled.");
                } else {
                    ROS_WARN("Mode OFFBOARD could not be enabled. Cannot execute moveit trajectory.");
                    return;
                }
            }

            auto trajectory = goal->trajectory;
            geometry_msgs::PoseStamped cmd_pose;
            for (const auto& p : trajectory.points) {
                if(action_server_.isPreemptRequested() || !ros::ok()){
                    ROS_INFO("Preempt requested");
                    success = false;
                    break;
                }
                geometry_msgs::Transform trans = p.transforms[0];
                cmd_pose.pose.position.x = trans.translation.x;
                cmd_pose.pose.position.y = trans.translation.y;
                cmd_pose.pose.position.z = trans.translation.z;
                cmd_pose.pose.orientation = trans.rotation;
                feedback_.current_pose = cmd_pose;
                local_pose_pub_.publish(cmd_pose);
                action_server_.publishFeedback(feedback_);
                ros::spinOnce();
                rate_.sleep();
            }

            tf::Pose tf_target;
            tf::poseMsgToTF(cmd_pose.pose, tf_target);
            while (!targetReached(tf_target))
            {
                local_pose_pub_.publish(cmd_pose);
                action_server_.publishFeedback(feedback_);
                feedback_.current_pose = cmd_pose;
                ros::spinOnce();
                rate_.sleep();
            }
        } else {
            ROS_WARN("Mavros not connected to FCU.");
            success = false;
        }

        // Set mode to loiter since keeping it in offboard requires sending commands
        // continuously
        mavros_msgs::SetMode loiter_set_mode;
        loiter_set_mode.request.custom_mode = "AUTO.LOITER";
        if (set_mode_client_.call(loiter_set_mode) && loiter_set_mode.response.mode_sent) {
            ROS_INFO("Mode AUTO.LOITER enabled.");
        } else {
            ROS_WARN("Mode AUTO.LOITER could not be enabled.");
            return;
        }

        if(success)
        {
          result_.error_code = Result::SUCCESSFUL;
          // set the action server to succeeded
          action_server_.setSucceeded(result_);
        }
        executing = false;
    }

    bool targetReached(const tf::Pose& target) {
        tf::Pose tf_curr;
        tf::poseMsgToTF(current_pose_.pose, tf_curr);
        auto diff_t = tf_curr.inverseTimes(target);
        if (fabsf(diff_t.getOrigin().x()) <= target_pos_tol && 
            fabsf(diff_t.getOrigin().y()) <= target_pos_tol &&
            fabsf(diff_t.getOrigin().z()) <= target_pos_tol &&
            fabsf(diff_t.getRotation().x()) <= target_orientation_tol &&
            fabsf(diff_t.getRotation().y()) <= target_orientation_tol &&
            fabsf(diff_t.getRotation().z()) <= target_orientation_tol &&
            fabsf(diff_t.getRotation().w()) - 1.0 <= target_orientation_tol)
        {
            return true;
        }
        return false;
    }

    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
    }

private:
    ros::NodeHandle nh_; // node handle

    ActionServer action_server_; // simple actionlib server
    Feedback feedback_; // action server feedback
    Result result_; // action server result
    std::string action_name_; // action name

    mavros_msgs::State current_state_; // latest mavros state
    geometry_msgs::PoseStamped current_pose_; // latest robot pose
    ros::Rate rate_ = {ros::Rate(20.0)};  // ros run rate
    ros::Subscriber local_pose_sub_; // mavros local position subscriber
    ros::Publisher local_pose_pub_; // mavros position commands publisher
    ros::Subscriber state_sub_; // mavros state subscriber 
    ros::ServiceClient arming_client_; // mavros service for arming/disarming the robot
    ros::ServiceClient set_mode_client_; // mavros service for setting mode. Position commands are only available in mode OFFBOARD.

    bool executing = {false}; // whether the action server is currently in execution
    
    const float target_pos_tol = {1e-1};
    const float target_orientation_tol = {5e-2};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FollowMultiDofJointTrajectoryActionServer");
  FollowMultiDofJointTrajectoryAction action("FollowMultiDofJointTrajectoryAction");
  action.idle();
  return 0;
}
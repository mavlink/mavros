#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <std_msgs/String.h>


geometry_msgs::PoseStamped current_pos;
void local_pos_gazebo_sub_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    current_pos.pose.position.x = msg->pose[1].position.x;
    current_pos.pose.position.y = msg->pose[1].position.y;
    current_pos.pose.position.z = msg->pose[1].position.z;
    current_pos.pose.orientation.x = msg->pose[1].orientation.x;
    current_pos.pose.orientation.y = msg->pose[1].orientation.y;
    current_pos.pose.orientation.z = msg->pose[1].orientation.z;
    current_pos.pose.orientation.w = msg->pose[1].orientation.w;

}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber local_pos_gazebo_sub = nh.subscribe<gazebo_msgs::ModelStates>
            ("gazebo/model_states", 10, local_pos_gazebo_sub_cb);
    ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros_extras/mocap/pose", 100);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    ros::Rate local_rate(50.0);


    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //  mavros_msgs::SetMode offb_set_mode;
    //  offb_set_mode.request.custom_mode = "OFFBOARD";

    //  mavros_msgs::CommandBool arm_cmd;
    //  arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.success){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        //    } 
        // else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }
         mocap_pos_pub.publish(current_pos);
         ros::spinOnce();
         local_rate.sleep();


        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

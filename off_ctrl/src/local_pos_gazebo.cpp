#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <std_msgs/String.h>


void pos_cb(const gazebo_msgs::ModelStates::ConstPtr &_msg)
{
    /* code for loop body */
    std::cout << _msg->pose[1].position.x << std::endl;
    std::cout << _msg->pose[1].position.y << std::endl; 
    std::cout << _msg->pose[1].position.z << std::endl;

    geometry_msgs::PoseStamped current_pos1;

    current_pos1.pose.position.x = _msg->pose[1].position.x;
    current_pos1.pose.position.y = _msg->pose[1].position.y;
    current_pos1.pose.position.z = _msg->pose[1].position.z;

    std::cout << "current_pos1" << std::endl;
    std::cout << current_pos1.pose.position.x << std::endl;
    std::cout << current_pos1.pose.position.y << std::endl; 
    std::cout << current_pos1.pose.position.z << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_pos_gazebo");
    ros::NodeHandle nh_gazebo;

    ros::Subscriber model_sub = nh_gazebo.subscribe<gazebo_msgs::ModelStates>
            ("/gazebo/model_states", 100, pos_cb);

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}
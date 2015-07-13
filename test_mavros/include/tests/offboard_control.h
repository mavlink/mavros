/**
 * @brief Offboard control test
 * @file offboard_control.h
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Andre Nguyen <andre-phu-van.nguyen@polymtl.ca>
 *
 * @addtogroup sitl_test
 * @{
 */
/*
 * Copyright 2015 Nuno Marques, Andre Nguyen.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros.h>
#include <sitl_test/sitl_test.h>
#include <sitl_test/test_type.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

namespace testtype {
/**
 * @brief Offboard controller tester
 *
 * Tests offboard position, velocity and acceleration control
 *
 */
class OffboardControl {
public:
    OffboardControl() :
        nh_sp("~"),
        mode("position"),
        shape("square")
    { };

    /* -*- main routine -*- */

    void spin(int argc, char *argv[]) {
        /**
         * @brief Setpoint control mode selector
         *
         * Available modes:
         * - position
         * - velocity
         * - acceleration
         */
        nh_sp.param<std::string>("mode", mode, "position");

        /**
         * @brief Setpoint path shape selector
         *
         * Available shapes:
         * - square
         * - circle
         * - eight
         * - ellipse (in 3D space)
         */
        nh_sp.param<std::string>("shape", shape, "square");

        ros::Rate loop_rate(10);

        ROS_INFO("SITL Test: Offboard control test running!");

        if(mode.compare("position") == 0){
            ROS_INFO("Position control mode selected.");

            if(shape.compare("square") == 0){
                ROS_INFO("Test option: square-shaped path...");
                uint8_t pos_target = 0;
                square_path_motion(loop_rate, pos_target);
            }
            
            else if(shape.compare("circle") == 0){
                ROS_INFO("Test option: circle-shaped path...");
                circle_path_motion(loop_rate);
                return;
            }

            else if(shape.compare("eight") == 0){
                ROS_INFO("Test option: eight-shaped path...");
                eight_path_motion(loop_rate);
            }

            else if(shape.compare("ellipse") == 0){
                ROS_INFO("Test option: ellipse-shaped path...");
                ellipse_path_motion(loop_rate);
                return;
            }
        }     
    }

private:
    ros::NodeHandle nh_sp;
    ros::Publisher local_pos_sp_pub;
    ros::Subscriber local_pos_sub;

    std::string mode;
    std::string shape;

    geometry_msgs::PoseStamped localpos;
    geometry_msgs::PoseStamped ps;

    /* -*- alias -*- */
    #define X  ps.pose.position.x
    #define Y  ps.pose.position.y
    #define Z  ps.pose.position.z
    #define sp_X  set_point().x
    #define sp_Y  set_point().y
    #define sp_Z  set_point().z

    /* -*- helper functions -*- */

    /**
     * @brief Defines single setpoint
     */
    geometry_msgs::Point set_point(){
        geometry_msgs::Point sp;
        /** @todo Give possibility to user define amplitude of movement (square corners coordinates)*/
        sp.x = 2.0f;
        sp.y = 2.0f;
        sp.z = 1.0f;
        return sp;
    }

    /**
     * @brief Defines circle path
     */
    geometry_msgs::PoseStamped circle_shape(int angle){
        geometry_msgs::PoseStamped sp;
        double r = 5.0f; // 5 meters radius

        sp.pose.position.x = r*cos(angle * M_PI/180.0f);
        sp.pose.position.y = r*sin(angle * M_PI/180.0f);
        sp.pose.position.z = 1.0f;

        return sp;
    }

    /**
     * @brief Defines Gerono lemniscate path
     */
    geometry_msgs::PoseStamped eight_shape(int angle){
        geometry_msgs::PoseStamped sp;
        double a = 5.0f; // vertical tangent with 5 meters size

        sp.pose.position.x = a*cos(angle * M_PI/180.0f);
        sp.pose.position.y = a*sin(angle * M_PI/180.0f) * cos(angle * M_PI/180.0f);
        sp.pose.position.z = 1.0f;

        return sp;
    }

    /**
     * @brief Defines ellipse path
     */
    geometry_msgs::PoseStamped ellipse_shape(int angle){
        geometry_msgs::PoseStamped sp;
        double a = 5.0f; // major axis
        double b = 2.0f; // minor axis

        // using spherical coordinates (rotation around y-axis)
        sp.pose.position.x = a*cos(angle * M_PI/180.0f);
        sp.pose.position.y = 0.0f;
        sp.pose.position.z = 2.5f + b*sin(angle * M_PI/180.0f);

        return sp;
    }

    /**
     * @brief Square path motion routine
     */
    void square_path_motion(ros::Rate loop_rate, uint8_t pos_target){
        local_pos_sp_pub = nh_sp.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

        while(ros::ok()){

            ROS_INFO("Testing...");

            local_pos_sp_pub.publish(ps);
            wait_destination(ps);

            // motion routine
            switch(pos_target) {
                case 1:
                    X = sp_X; Y = sp_Y; Z = sp_Z;
                    break;
                case 2:
                    X = -sp_X; Y = sp_Y; Z = sp_Z;
                    break;
                case 3:
                    X = -sp_X; Y = -sp_Y; Z = sp_Z;
                    break;
                case 4:
                    X = sp_X; Y = -sp_Y; Z = sp_Z;
                    break;
                case 5:
                    X = sp_X; Y = sp_Y; Z = sp_Z;
                    break;
                default:
                    break;
            }
            if (pos_target == 6) {
                ROS_INFO("Test complete!");
                ros::shutdown();
            }
            else
                ++pos_target;
            
            loop_rate.sleep();
            ros::spinOnce();
        }
    }

    /**
     * @brief Circle path motion routine
     */
    void circle_path_motion(ros::Rate loop_rate){
        local_pos_sp_pub = nh_sp.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        
        ROS_INFO("Testing...");

        while(ros::ok()){
            // starting point
            X = 5.0f;
            Y = 0.0f;
            Z = 1.0f;
            local_pos_sp_pub.publish(ps);
            wait_destination(ps);

            // motion routine
            for(int theta = 0; theta <= 360; theta++){
                local_pos_sp_pub.publish(circle_shape(theta));
                if (theta == 360){
                    ROS_INFO("Test complete!");
                    ros::shutdown();
                }
                loop_rate.sleep();
                ros::spinOnce();
            }                
        }
    }

    /**
     * @brief Eight path motion routine
     */
    void eight_path_motion(ros::Rate loop_rate){
        local_pos_sp_pub = nh_sp.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        
        ROS_INFO("Testing...");

        while(ros::ok()){
            // starting point
            X = 5.0f;
            Y = 0.0f;
            Z = 1.0f;
            local_pos_sp_pub.publish(ps);
            wait_destination(ps);

            // motion routine
            for(int theta = -180; theta <= 180; theta++){
                local_pos_sp_pub.publish(eight_shape(theta));
                if (theta == 360){
                    ROS_INFO("Test complete!");
                    ros::shutdown();
                }
                loop_rate.sleep();
                ros::spinOnce();
            }                
        }
    }

    /**
     * @brief Ellipse path motion routine
     */
    void ellipse_path_motion(ros::Rate loop_rate){
        local_pos_sp_pub = nh_sp.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        
        ROS_INFO("Testing...");

        while(ros::ok()){
            // starting point
            X = 0.0f;
            Y = 0.0f;
            Z = 2.5f;
            local_pos_sp_pub.publish(ps);
            wait_destination(ps);

            // motion routine
            for(int theta = 0; theta <= 360; theta++){
                local_pos_sp_pub.publish(ellipse_shape(theta));
                if (theta == 360){
                    ROS_INFO("Test complete!");
                    ros::shutdown();
                }
                loop_rate.sleep();
                ros::spinOnce();
            }                
        }
    }

    /**
     * @brief Defines the accepted threshold to the destination/target position
     * before moving to the next setpoint.
     */
    void wait_destination(geometry_msgs::PoseStamped target){
        bool stop = false;
        ros::Rate loop_rate(10);

        local_pos_sub = nh_sp.subscribe("/mavros/local_position/local", 10, &OffboardControl::local_pos_cb, this);
        geometry_msgs::Point pos = localpos.pose.position, dest = target.pose.position;

        while (ros::ok() && !stop){
            double distance = sqrt(
                    (dest.x - pos.x)*(dest.x - pos.x) +
                    (dest.y - pos.y)*(dest.y - pos.y) +
                    (dest.z - pos.z)*(dest.z - pos.z));
            
            if(distance <= 0.1f) /** @todo Add gaussian threshold */
                stop = true;

            local_pos_sp_pub.publish(target);
            ros::spinOnce();

            pos = localpos.pose.position;
            loop_rate.sleep();
        }
    }

    /* -*- callbacks -*- */

    void local_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg){
        localpos = *msg;
    }
};
};  // namespace testype

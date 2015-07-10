/**
 * @brief Offboard position control test - square movement
 * @file offboard_posctl_square.h
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
 * @brief Offboard position control test - square movement
 *
 * Tests offboard position control, making a square on the
 * open world Gazebo map.
 *
 * @todo Set this a generic Offboard position setpoint tester;
 * give possibility to define type of motion in a parameter
 * (p.e: circle, rectangle, eight, spiral)
 */
class OffboardPosCtrlSquare {
public:
    OffboardPosCtrlSquare() :
        n("~offboard_pos_ctrl_sq")
    { };

    /* -*- main routine -*- */

    void spin(int argc, char *argv[]) {

        ROS_INFO("SITL Test: Offboard control test running!");
        ROS_INFO("Test option: square shape movement...");

        ros::Rate loop_rate(10);
        uint8_t pos_target = 0;
        set_motion(loop_rate, pos_target);
    }

private:
    ros::NodeHandle n;

    ros::Publisher local_position_sp_pub;
    ros::Subscriber local_pos_sub;
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
     * @brief Defines setpoint
     */
    geometry_msgs::Point set_point(){
        geometry_msgs::Point sp;
        /** @todo Give possibility to user define amplitude of movement*/
        sp.x = 2.0f;
        sp.y = 2.0f;
        sp.z = 1.0f;
        return sp;
    }

    /**
     * @brief Motion routine
     */
    void set_motion(ros::Rate loop_rate, uint8_t pos_target){

        local_position_sp_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        local_pos_sub = n.subscribe("/mavros/local_position/local", 10, &OffboardPosCtrlSquare::local_pos_cb, this) ;

        while(ros::ok()){

            ROS_INFO("Testing...");

            local_position_sp_pub.publish(ps);
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
     * @brief Defines the accepted threshold to the setpointed position
     * before moving to the next setpoint.
     */
    void wait_destination(geometry_msgs::PoseStamped destination){
        bool stop = false;
        ros::Rate loop_rate(10);

        geometry_msgs::Point pos = localpos.pose.position, dest = destination.pose.position;

        while (ros::ok() && !stop){
            double distance = sqrt(
                    pow(dest.x - pos.x, 2) +
                    pow(dest.y - pos.y, 2) +
                    pow(dest.z - pos.z, 2)
            );
            
            if(distance <= 0.1f){ /** @todo Add gaussian threshold */
                stop = true;
            }

            local_position_sp_pub.publish(destination);
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

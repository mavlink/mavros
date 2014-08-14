#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include "setpoint_mixin.h"

namespace mavplugin {


class MocapPosePlugin : public MavRosPlugin,
    private TFListenerMixin<MocapPosePlugin>
{
public:
    MocapPosePlugin() :
        uas(nullptr)
    { };

    void initialize(UAS &uas_,
            ros::NodeHandle &nh,
            diagnostic_updater::Updater &diag_updater)
    {
        bool use_tf;
        bool use_pose;

        uas = &uas_;
        sp_nh = ros::NodeHandle(nh, "mocap_pose");

        sp_nh.param("mocap/use_tf", use_tf, false);  // Vicon
        sp_nh.param("mocap/use_pose", use_pose, true);  // Optitrack


        if (use_tf && !use_pose)
        {
            mocap_tf_sub = sp_nh.subscribe("mocap/tf", 1, &MocapPosePlugin::mocap_tf_cb, this);
        }

        if (use_pose && !use_tf)
        {
            mocap_pose_sub = sp_nh.subscribe("mocap/pose", 1, &MocapPosePlugin::mocap_pose_cb, this);
        }
				else
				{
						ROS_ERROR_THROTTLE(1, "Use one motion capture source.");
				}
    }

    const std::string get_name() const
    {
        return "MocapPose";
    }

    const message_map get_rx_handlers()
    {
        return { /* Rx disabled */ };
    }

private:
    friend class TFListenerMixin;
    UAS *uas;

    ros::NodeHandle sp_nh;
    ros::Subscriber mocap_pose_sub;
    ros::Subscriber mocap_tf_sub;

    // mavlink send

    void mocap_pose_send
            (uint64_t usec,
            float x, float y, float z,
            float roll, float pitch, float yaw)
    {
        mavlink_message_t msg;
        mavlink_msg_vicon_position_estimate_pack_chan(UAS_PACK_CHAN(uas), &msg,
                usec,
                x,
                y,
                z,
                roll,
                pitch,
                yaw);
        uas->mav_link->send_message(&msg);
    }


    void mocap_pose_cb(const geometry_msgs::Pose::ConstPtr &pose)
    {	
				ros::Time stamp = ros::Time::now();
				tf::Quaternion quat;
        tf::quaternionMsgToTF(pose->orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
				// Covert to malink body frame
        mocap_pose_send(stamp.toNSec() / 1000,
                pose->position.x,
							  -pose->position.y,
							  -pose->position.z,
                roll, -pitch, -yaw); 
    }

    void mocap_tf_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
    {
				tf::Quaternion quat;
        tf::quaternionMsgToTF(trans->transform.rotation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
				// Covert to malink body frame
        mocap_pose_send(trans->header.stamp.toNSec() / 1000,
                trans->transform.translation.x,
							  -trans->transform.translation.y,
							  -trans->transform.translation.z,
                roll, -pitch, -yaw); 
    }
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::MocapPosePlugin, mavplugin::MavRosPlugin)

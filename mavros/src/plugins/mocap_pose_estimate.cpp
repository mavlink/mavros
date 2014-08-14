#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>


namespace mavplugin {

class MocapPosePlugin : public MavRosPlugin
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
        mp_nh = ros::NodeHandle(nh, "mocap_pose");

        mp_nh.param("mocap/use_tf", use_tf, false);  // Vicon
        mp_nh.param("mocap/use_pose", use_pose, true);  // Optitrack


        if (use_tf && !use_pose)
        {
            mocap_tf_sub = mp_nh.subscribe("mocap/tf", 1, &MocapPosePlugin::mocap_tf_cb, this);
        }

        if (use_pose && !use_tf)
        {
            mocap_pose_sub = mp_nh.subscribe("mocap/pose", 1, &MocapPosePlugin::mocap_pose_cb, this);
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
    UAS *uas;

    ros::NodeHandle mp_nh;
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
        // Convert to mavlink body frame
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
        // Convert to mavlink body frame
        mocap_pose_send(trans->header.stamp.toNSec() / 1000,
                trans->transform.translation.x,
                -trans->transform.translation.y,
                -trans->transform.translation.z,
                roll, -pitch, -yaw); 
    }
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::MocapPosePlugin, mavplugin::MavRosPlugin)

/**
 * @brief JoyControl plugin
 * @file joy_control.cpp
 * @author Francois Chataigner <chataign@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/Joy.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

#include <thread>
#include <mutex>
using namespace std;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template<typename Service>
bool call_service( const string& service_name, const typename Service::Request& req )
{
	if ( !ros::service::exists( service_name, true ) )
		return false;

	Service cmd;
	cmd.request = req;

	bool success = ros::service::call( service_name, cmd ) && cmd.response.success;
	ROS_DEBUG_STREAM( "calling service=" << service_name << " success=" << success );
	return success;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

namespace mavplugin {
/**
 * @brief Joystick Control plugin
 */
class JoyControlPlugin : public MavRosPlugin
{
	struct Control
	{
		int axis;
		double scale;
		double offset;
		double deadzone;

		double map_axis( const sensor_msgs::Joy::ConstPtr& joy ) const
		{
			if ( axis < 0 || axis >= joy->axes.size() ) throw std::out_of_range("failed to map axis");
			double val = joy->axes[axis] + offset;
			return ( val > deadzone || val < -deadzone ? val * scale : 0.0 );
		}
	};

	Control x_, y_, z_, r_;

	int arm_button_;
	vector<int> mode_mapping_;
	vector<string> mode_names_;

	string arming_service_;
	string mode_service_;

public:
	JoyControlPlugin() :
		nh_("~joy_control"),
		uas_(nullptr) {}

	void initialize(UAS &uas )
	{
		uas_ = &uas;

		int rc_rate_hz;

		nh_.param("rc_rate_hz",     rc_rate_hz,      10 );

		nh_.param("arming_service", arming_service_, string("/mavros/cmd/arming") );
		nh_.param("mode_service",   mode_service_,   string("/mavros/set_mode") );

		// Mode switches on remotes can be reported a single flag, or as a binary
		// combination of several flags: eg. mode1=[0,0]=0, mode2=[1,0]=1, mode3=[0,1]=2
		// mode_mapping={3,4} means a binary combination of the 4th and 5th values
		// in the sensor_msgs::Joy buttons array

		nh_.param("mode_mapping",   mode_mapping_,  {3,4} );
		nh_.param("mode_names",     mode_names_,    {"MANUAL","ALTCTL","OFFBOARD"} );

		nh_.param("arm_button",     arm_button_,    2   );

		nh_.param("x_axis",         x_.axis,        1   );
		nh_.param("x_scale",        x_.scale,       1.0 );
		nh_.param("x_offset",       x_.offset,      0.0 );
		nh_.param("x_deadzone",     x_.deadzone,    0.2 );

		nh_.param("y_axis",         y_.axis,        0   );
		nh_.param("y_scale",        y_.scale,       1.0 );
		nh_.param("y_offset",       y_.offset,      0.0 );
		nh_.param("y_deadzone",     y_.deadzone,    0.2 );

		nh_.param("z_axis",         z_.axis,        2   );
		nh_.param("z_scale",        z_.scale,       1.0 );
		nh_.param("z_offset",       z_.offset,      0.0 );
		nh_.param("z_deadzone",     z_.deadzone,    0.0 );

		nh_.param("r_axis",         r_.axis,        4   );
		nh_.param("r_scale",        r_.scale,       1.0 );
		nh_.param("r_offset",       r_.offset,      0.0 );
		nh_.param("r_deadzone",     r_.deadzone,    0.2 );

		joystick_sub_ = nh_.subscribe("/joy", 10, &JoyControlPlugin::handle_joystick, this );
		state_sub_ = nh_.subscribe("/mavros/state", 10, &JoyControlPlugin::handle_state, this );

		thread_.reset( new std::thread( &JoyControlPlugin::thread_call, this, rc_rate_hz ) );
	}

	const message_map get_rx_handlers() {
		return { };
	}

private:
	ros::NodeHandle nh_;
	UAS *uas_;
	ros::Subscriber joystick_sub_, state_sub_;
	mavros_msgs::State state_;

	std::mutex msg_mutex_;
	shared_ptr<std::thread> thread_;
	mavlink_message_t ctrl_msg_;

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void thread_call( float rate_hz )
	{
		ros::Rate rate(rate_hz);

		// We need to keep sending messages at a fixed
		// rate or PX4 will trigger RC timeouts

		while (1)
		{
			msg_mutex_.lock();
			UAS_FCU(uas_)->send_message(&ctrl_msg_);
			msg_mutex_.unlock();

			rate.sleep();
		}
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	uint16_t array_to_bitset( const vector<int32_t>& array )
	{
		uint16_t bits = 0;

		for ( size_t i = 0; i < array.size(); ++i )
			if ( array[i] > 0 ) bits |= ( 1 << i );

		return bits;
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void handle_joystick( const sensor_msgs::Joy::ConstPtr& joy )
	{
		try
		{
			// PX4 seems to ignore the buttons field when processing manual
			// control messages so we need to handle button input ourselves
			// @see MavlinkReceiver::handle_message_manual_control()

			handle_arming   ( joy );// toggle arm/disarm based on /mavros/state
			handle_mode     ( joy );

			float x = x_.map_axis( joy );
			float y = y_.map_axis( joy );
			float z = z_.map_axis( joy );
			float r = r_.map_axis( joy );

			std::lock_guard<std::mutex> lock(msg_mutex_);

			mavlink_msg_manual_control_pack_chan( UAS_PACK_CHAN(uas_), &ctrl_msg_,
					(uas_)->get_tgt_system(), x, y, z, r, array_to_bitset(joy->buttons) );
		}
		catch ( const std::out_of_range& e )
		{
			ROS_WARN( "failed to process joystick input: %s", e.what() );
		}
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void handle_arming( const sensor_msgs::Joy::ConstPtr& joy )
	{
		if ( arm_button_ < 0 || arm_button_ > joy->buttons.size() )
			throw std::out_of_range("arm button index is invalid");

		bool arm_button_on = joy->buttons[arm_button_];

		if ( arm_button_on && !state_.armed )
		{
			mavros_msgs::CommandBool::Request req;
			req.value = true;
			call_service<mavros_msgs::CommandBool>( arming_service_, req );
		}
		else if ( arm_button_on && state_.armed )
		{
			mavros_msgs::CommandBool::Request req;
			req.value = false;
			call_service<mavros_msgs::CommandBool>( arming_service_, req );
		}
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void handle_mode( const sensor_msgs::Joy::ConstPtr& joy )
	{
		int mode_idx = 0;

		// find the mode index as a binary combination of the multiple flags
		// eg. [0,1] <-> index=2, [1,1] <-> index=3

		for ( int i = 0; i < mode_mapping_.size(); ++i )
			mode_idx += pow(2,i) * joy->buttons.at( mode_mapping_[i] );

		static mavros_msgs::SetMode::Request req;
		req.custom_mode = mode_names_.at(mode_idx);

		if ( req.custom_mode != state_.mode )
		{
			ROS_INFO_STREAM("setting mode=" << req.custom_mode);
			call_service<mavros_msgs::SetMode>( mode_service_, req );
		}
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void handle_state( const mavros_msgs::State::ConstPtr& msg )
	{
		state_ = *msg;
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::JoyControlPlugin, mavplugin::MavRosPlugin)


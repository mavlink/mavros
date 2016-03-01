/**
 * @brief JoyControl plugin
 * @file joy_control.cpp
 * @author Francois Chataigner <chataign@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 Francois Chataigner.
 *
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

using namespace std;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

namespace mavplugin {
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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Joystick Control plugin
 */
class JoyControlPlugin : public MavRosPlugin
{
	ros::NodeHandle mavros_nh_, plugin_nh_;
	UAS *uas_;
	mavros_msgs::State state_;
	mavlink_message_t ctrl_msg_;

	Control x_, y_, z_, r_;

	int arm_button_;
	vector<int> mode_mapping_;
	vector<string> mode_names_;

	ros::Timer send_msg_timer_;
	ros::ServiceClient arming_srv_, set_mode_srv_;
	ros::Subscriber joystick_sub_, state_sub_;

public:
	JoyControlPlugin()
		: mavros_nh_("~")
		, plugin_nh_("~joy_control")
		, uas_(nullptr) {}

	void initialize(UAS &uas )
	{
		uas_ = &uas;

		int msg_rate;

		plugin_nh_.param<int>("msg_rate", msg_rate, 10 );
		plugin_nh_.param<int>("arm_button", arm_button_, 2 );

		// Mode switches on remotes can be reported a single flag, or as a binary
		// combination of several flags: eg. mode1=[0,0]=0, mode2=[1,0]=1, mode3=[0,1]=2
		// mode_mapping={3,4} means a binary combination of the 4th and 5th values
		// in the sensor_msgs::Joy buttons array

		plugin_nh_.param< vector<int> >("mode_mapping", mode_mapping_, {3,4} );
		plugin_nh_.param< vector<string> >("mode_names", mode_names_, {"MANUAL","ALTCTL","OFFBOARD"} );

		plugin_nh_.param<int>      ("x_axis",       x_.axis,        1   );
		plugin_nh_.param<double>   ("x_scale",      x_.scale,       1.0 );
		plugin_nh_.param<double>   ("x_offset",     x_.offset,      0.0 );
		plugin_nh_.param<double>   ("x_deadzone",   x_.deadzone,    0.2 );

		plugin_nh_.param<int>      ("y_axis",       y_.axis,        0   );
		plugin_nh_.param<double>   ("y_scale",      y_.scale,       1.0 );
		plugin_nh_.param<double>   ("y_offset",     y_.offset,      0.0 );
		plugin_nh_.param<double>   ("y_deadzone",   y_.deadzone,    0.2 );

		plugin_nh_.param<int>      ("z_axis",       z_.axis,        2   );
		plugin_nh_.param<double>   ("z_scale",      z_.scale,       1.0 );
		plugin_nh_.param<double>   ("z_offset",     z_.offset,      0.0 );
		plugin_nh_.param<double>   ("z_deadzone",   z_.deadzone,    0.0 );

		plugin_nh_.param<int>      ("r_axis",       r_.axis,        4   );
		plugin_nh_.param<double>   ("r_scale",      r_.scale,       1.0 );
		plugin_nh_.param<double>   ("r_offset",     r_.offset,      0.0 );
		plugin_nh_.param<double>   ("r_deadzone",   r_.deadzone,    0.2 );

		arming_srv_ = mavros_nh_.serviceClient<mavros_msgs::CommandBool>("cmd/arming");
		set_mode_srv_ = mavros_nh_.serviceClient<mavros_msgs::SetMode>("set_mode");

		arming_srv_.waitForExistence();
		//set_mode_srv_.waitForExistence();

		joystick_sub_ = mavros_nh_.subscribe("joy_topic", 10, &JoyControlPlugin::handle_joystick, this );
		state_sub_ = mavros_nh_.subscribe("state", 10, &JoyControlPlugin::handle_state, this );
		send_msg_timer_ = mavros_nh_.createTimer( ros::Rate(msg_rate), &JoyControlPlugin::send_message, this );
	}

	const message_map get_rx_handlers() {
		return { };
	}

private:

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

			mavlink_msg_manual_control_pack_chan( UAS_PACK_CHAN(uas_), &ctrl_msg_,
					(uas_)->get_tgt_system(), x, y, z, r, array_to_bitset(joy->buttons) );
		}
		catch ( const std::out_of_range& e )
		{
			ROS_ERROR( "failed to process joystick input: %s", e.what() );
		}
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void send_message( const ros::TimerEvent& ) {
		UAS_FCU(uas_)->send_message(&ctrl_msg_);
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
			mavros_msgs::CommandBool arming;
			arming.request.value = true;
			arming_srv_.call(arming);
		}
		else if ( arm_button_on && state_.armed )
		{
			mavros_msgs::CommandBool arming;
			arming.request.value = false;
			arming_srv_.call(arming);
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

		static mavros_msgs::SetMode set_mode;
		set_mode.request.custom_mode = mode_names_.at(mode_idx);

		if ( set_mode.request.custom_mode != state_.mode )
		{
			ROS_INFO_STREAM("setting custom mode=" << set_mode.request.custom_mode);
			set_mode_srv_.call( set_mode );
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


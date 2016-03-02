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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

namespace mavplugin {
/**
 * @brief Properties for joystick control axis
 */
struct JoyControl
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

	void load_params( ros::NodeHandle &priv_nh, const std::string& name,
			int def_axis = 0, double def_scale = 1, double def_offset = 0, double def_deadzone = 0 )
	{
		auto nh = ros::NodeHandle( priv_nh, name );

		nh.param( "axis",		axis,		def_axis		);
		nh.param( "scale",		scale,		def_scale		);
		nh.param( "offset",		offset,		def_offset		);
		nh.param( "deadzone",	deadzone,	def_deadzone	);
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
	mavros_msgs::State::ConstPtr curr_state_;

	JoyControl x_, y_, z_, r_;

	int arm_button_;
	std::vector<int> mode_mapping_;
	std::vector<std::string> mode_names_;

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

		// Mode switches on remotes can be reported a single flag, or as a binary
		// combination of several flags: eg. mode1=[0,0]=0, mode2=[1,0]=1, mode3=[0,1]=2
		// mode_mapping={3,4} means a binary combination of the 4th and 5th values
		// in the sensor_msgs::Joy buttons array.

		// Note: param<> template type must be specified
		// here for compatibility with older compilers.

		plugin_nh_.param< std::vector<int>			>("mode_mapping", mode_mapping_, {3,4} );
		plugin_nh_.param< std::vector<std::string>  >("mode_names", mode_names_, {"MANUAL","ALTCTL","OFFBOARD"} );

		plugin_nh_.param("arm_button", arm_button_, 2 );

		x_.load_params( plugin_nh_, "x", 1, 1.0, 0.0, 0.2 );
		y_.load_params( plugin_nh_, "y", 0, 1.0, 0.0, 0.2 );
		z_.load_params( plugin_nh_, "z", 2, 1.0, 0.0, 0.0 );
		r_.load_params( plugin_nh_, "r", 4, 1.0, 0.0, 0.2 );

		arming_srv_ = mavros_nh_.serviceClient<mavros_msgs::CommandBool>("cmd/arming");
		set_mode_srv_ = mavros_nh_.serviceClient<mavros_msgs::SetMode>("set_mode");

		joystick_sub_ = mavros_nh_.subscribe("joy_topic", 10, &JoyControlPlugin::handle_joystick, this );
		state_sub_ = mavros_nh_.subscribe("state", 10, &JoyControlPlugin::handle_state, this );
	}

	const message_map get_rx_handlers() {
		return { };
	}

private:

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	uint16_t array_to_bitset( const std::vector<int32_t>& array )
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
			handle_arming( joy );
			handle_mode( joy );

			// PX4 seems to ignore the buttons field when processing manual
			// control messages so we need to handle button input ourselves
			// @see MavlinkReceiver::handle_message_manual_control()

			float x = x_.map_axis( joy );
			float y = y_.map_axis( joy );
			float z = z_.map_axis( joy );
			float r = r_.map_axis( joy );

			mavlink_message_t ctrl_msg;

			mavlink_msg_manual_control_pack_chan( UAS_PACK_CHAN(uas_), &ctrl_msg,
					(uas_)->get_tgt_system(), x, y, z, r, array_to_bitset(joy->buttons) );

			UAS_FCU(uas_)->send_message(&ctrl_msg);
		}
		catch ( const std::out_of_range& e )
		{
			ROS_ERROR( "failed to process joystick input: %s", e.what() );
		}
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void handle_arming( const sensor_msgs::Joy::ConstPtr& joy )
	{
		if ( !curr_state_ )
		{
			ROS_WARN_THROTTLE( 1, "need message on state topic to arm/disarm" );
			return;
		}

		if ( arm_button_ < 0 || arm_button_ > joy->buttons.size() )
			throw std::out_of_range("arm button index is invalid");

		bool arm_button_on = joy->buttons[arm_button_];
		if ( !arm_button_on ) return;	// nothing to do

		mavros_msgs::CommandBool arming;
		arming.request.value = !curr_state_->armed;

		if ( !arming_srv_.call(arming) || !arming.response.success )
			ROS_WARN("call to arm/disarm service failed with result=%d", arming.response.result );
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void handle_mode( const sensor_msgs::Joy::ConstPtr& joy )
	{
		if ( !curr_state_ )
		{
			ROS_WARN_THROTTLE( 1, "need message on state topic to change mode" );
			return;
		}

		int mode_idx = 0;

		// find the mode index as a binary combination of the multiple flags
		// eg. [0,1] <-> index=2, [1,1] <-> index=3

		for ( int i = 0; i < mode_mapping_.size(); ++i )
			mode_idx += pow(2,i) * joy->buttons.at( mode_mapping_[i] );

		mavros_msgs::SetMode set_mode;
		set_mode.request.custom_mode = mode_names_.at(mode_idx);

		if ( set_mode.request.custom_mode == curr_state_->mode )
			return;	// already in requested mode

		ROS_INFO_STREAM( "setting custom mode=" << set_mode.request.custom_mode );

		if ( !set_mode_srv_.call(set_mode) || !set_mode.response.success )
			ROS_WARN_STREAM("failed to set custom mode");
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void handle_state( const mavros_msgs::State::ConstPtr& state )
	{
		curr_state_ = state;
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::JoyControlPlugin, mavplugin::MavRosPlugin)


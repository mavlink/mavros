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
struct JoyAxis
{
	int axis;
	double scale;
	double offset;
	double deadzone;

	double map_axis( const sensor_msgs::Joy& joy_msg ) const
	{
		if ( axis < 0 || axis >= joy_msg.axes.size() ) throw std::out_of_range("failed to map axis");
		double val = joy_msg.axes[axis] + offset;
		return ( val > deadzone || val < -deadzone ? val * scale : 0.0 );
	}

	void load_params( ros::NodeHandle &nh, const std::string& axis_ns,
	                  int def_axis = 0, double def_scale = 1, double def_offset = 0, double def_deadzone = 0 )
	{
		auto axis_nh = ros::NodeHandle( nh, axis_ns );

		axis_nh.param( "axis",          axis,           def_axis                );
		axis_nh.param( "scale",         scale,          def_scale               );
		axis_nh.param( "offset",        offset,         def_offset              );
		axis_nh.param( "deadzone",      deadzone,       def_deadzone    );
	}
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct JoyButton
{
	typedef std::function< void () > Callback;
	typedef std::function< void ( const std::string& custom_mode ) > ModeCallback;

	std::vector<int> switches;
	std::vector<int> values;
	Callback callback;

	bool toggled( const sensor_msgs::Joy& joy_msg ) const // throws
	{
		for ( int i=0; i< switches.size(); ++i )
			if ( joy_msg.buttons.at( switches[i] ) != values.at(i) ) return false;
		return true;
	}

	void handle( const sensor_msgs::Joy& joy_msg ) {
		if ( toggled(joy_msg) ) callback();
	}

	bool load( ros::NodeHandle &nh, const std::string& button_ns, Callback new_callback )
	{
		callback = new_callback;
		auto button_nh = ros::NodeHandle( nh, button_ns );
		return button_nh.getParam( "switches", switches ) && button_nh.getParam( "values", values );
	}

	bool load( ros::NodeHandle &nh, const std::string& button_ns, ModeCallback mode_callback )
	{
		auto button_nh = ros::NodeHandle( nh, button_ns );

		std::string custom_mode;
		if ( !button_nh.getParam( "custom_mode", custom_mode ) ) return false;

		return load( nh, button_ns, (Callback) boost::bind( mode_callback, custom_mode ) );
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

	JoyAxis x_, y_, z_, r_;
	std::vector<JoyButton> buttons_;

	ros::ServiceClient arming_srv_, set_mode_srv_;
	ros::Subscriber joystick_sub_, state_sub_;

public:

	JoyControlPlugin()
		: mavros_nh_("~")
		, plugin_nh_("~joy_control")
		, uas_(nullptr) {
	}

	void initialize(UAS &uas )
	{
		uas_ = &uas;

		JoyButton::Callback arm_callback = boost::bind( &JoyControlPlugin::handle_arming, this );
		JoyButton::ModeCallback mode_callback = boost::bind( &JoyControlPlugin::handle_mode, this, _1 );

		JoyButton button;

		if ( button.load( plugin_nh_, "arm", arm_callback ) ) buttons_.push_back(button);
		else ROS_ERROR("failed to read 'arm' button");

		int num_modes=0;
		plugin_nh_.getParam( "num_modes", num_modes );

		for ( int i=0; i< num_modes; ++i )
		{
			std::string mode_ns = "mode" + std::to_string(i+1);
			if ( button.load( plugin_nh_, mode_ns, mode_callback ) ) buttons_.push_back(button);
			else ROS_ERROR_STREAM("failed to read button for mode=" << mode_ns );
		}

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

	void handle_joystick( const sensor_msgs::Joy& joy_msg )
	{
		try
		{
			for ( auto button : buttons_ ) button.handle(joy_msg);

			// PX4 seems to ignore the buttons field when processing manual
			// control messages so we need to handle button input ourselves
			// @see MavlinkReceiver::handle_message_manual_control()

			float x = x_.map_axis( joy_msg );
			float y = y_.map_axis( joy_msg );
			float z = z_.map_axis( joy_msg );
			float r = r_.map_axis( joy_msg );

			mavlink_message_t ctrl_msg;

			mavlink_msg_manual_control_pack_chan( UAS_PACK_CHAN(uas_), &ctrl_msg,
				                                  (uas_)->get_tgt_system(), x, y, z, r, array_to_bitset(joy_msg.buttons) );

			UAS_FCU(uas_)->send_message(&ctrl_msg);
		}
		catch ( const std::out_of_range& e )
		{
			ROS_ERROR( "failed to process joystick input: %s", e.what() );
		}
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void handle_arming()
	{
		if ( !curr_state_ )
		{
			ROS_WARN_THROTTLE( 1, "need message on state topic to arm/disarm" );
		}
		else
		{
			mavros_msgs::CommandBool arming;
			arming.request.value = !curr_state_->armed;

			if ( !arming_srv_.call(arming) || !arming.response.success )
				ROS_WARN("call to arm/disarm service failed with result=%d", arming.response.result );
		}
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void handle_mode( const std::string& custom_mode )
	{
		if ( !curr_state_ )
		{
			ROS_WARN_THROTTLE( 1, "need message on state topic to change mode" );
		}
		else if ( custom_mode != curr_state_->mode )
		{
			ROS_INFO_STREAM( "setting custom mode=" << custom_mode );

			mavros_msgs::SetMode set_mode;
			set_mode.request.custom_mode = custom_mode;

			if ( !set_mode_srv_.call(set_mode) || !set_mode.response.success )
				ROS_WARN_STREAM("failed to set custom mode");
		}
	}

	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////

	void handle_state( const mavros_msgs::State::ConstPtr& state )
	{
		curr_state_ = state;
	}
};
};      // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::JoyControlPlugin, mavplugin::MavRosPlugin)


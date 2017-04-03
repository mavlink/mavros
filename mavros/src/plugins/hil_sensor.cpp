/**
 * @brief HilSensor plugin
 * @file hil_sensor.cpp
 * @author Mohamed Abdelkader <mohamedashraf123@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 Mohamed Abdelkader.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/HilSensor.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief HIL Sensor plugin
 *
 * Send HIL sensor to FCU controller.
 */
class HilSensorPlugin : public plugin::PluginBase {
public:
	HilSensorPlugin() : PluginBase(),
		sensor_nh("~hil_sensor")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
<<<<<<< HEAD
=======
		last_time = ros::Time(0.0);
		sensor_period = ros::Duration(0.025);	// 40hz
>>>>>>> 2d5d64565cfcb504db61fd9bba431d0afd875584

		hilSensor_sub = sensor_nh.subscribe("imu_ned", 10, &HilSensorPlugin::sensor_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle sensor_nh;

	ros::Subscriber hilSensor_sub;
	ros::Time last_time;
	ros::Duration sensor_period;

  /* -*- low-level send -*- */
  //! Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_SENSOR
	void set_hil_sensor(uint64_t time_boot_us,
                        float xacc, float yacc, float zacc,
                        float xgyro, float ygyro, float zgyro,
                        float xmag, float ymag, float zmag,
                        float abs_pressure, float diff_pressure, float pressure_alt,
                        float temperature,
                        uint32_t fields_updated) {
		    mavlink::common::msg::HIL_SENSOR sensor;

        sensor.time_usec=time_boot_us;
        sensor.xacc=xacc;
        sensor.yacc=yacc;
        sensor.zacc=zacc;
        sensor.xgyro=xgyro;
        sensor.ygyro=ygyro;
        sensor.zgyro=zgyro;
        sensor.xmag=xmag;
        sensor.ymag=ymag;
        sensor.zmag=zmag;
        sensor.abs_pressure=abs_pressure;
        sensor.diff_pressure=diff_pressure;
        sensor.pressure_alt=pressure_alt;
        sensor.temperature=temperature;
        sensor.fields_updated=fields_updated;

		    UAS_FCU(m_uas)->send_message_ignore_drop(sensor);
	}

	/* -*- mid-level helpers -*- */
	/**
	 * @brief Send hil_sensor to FCU.
	 *
	 */
	void send_hil_sensor(const ros::Time &stamp,
                         float xacc, float yacc, float zacc,
                         float xgyro, float ygyro, float zgyro,
                         float xmag, float ymag, float zmag,
                         float abs_pressure, float diff_pressure, float pressure_alt,
                         float temperature,
                         uint32_t fields_updated) {
		      set_hil_sensor(stamp.toNSec() / 1000,
					xacc, yacc, zacc,
					xgyro, ygyro, zgyro,
					xmag, ymag, zmag,
					abs_pressure, diff_pressure, pressure_alt,
					temperature,
					fields_updated);
	}

	/* -*- callbacks -*- */
<<<<<<< HEAD
		void sensor_cb(const mavros_msgs::HilSensor::ConstPtr &req) {
            send_hil_sensor(req->header.stamp,
                            req->acc.x, req->acc.y, req->acc.z,
                            req->gyro.x, req->gyro.y, req->gyro.z,
                            req->mag.x, req->mag.y, req->mag.z,
                            req->abs_pressure.fluid_pressure, req->diff_pressure.fluid_pressure, req->pressure_alt.fluid_pressure,
                            req->temperature.temperature,
                            req->fields_updated);
=======
        

	void sensor_cb(const mavros_msgs::HilSensor::ConstPtr &req) {
		// Throttle incoming messages to 40hz
		if ((ros::Time::now() - last_time) < sensor_period) {
			return;
		}
		last_time = ros::Time::now();
            
		send_hil_sensor(req->header.stamp,
			    req->xacc, req->yacc, req->zacc,
			    req->xgyro, req->ygyro, req->zgyro,
			    req->xmag, req->ymag, req->zmag,
			    req->abs_pressure, req->diff_pressure, req->pressure_alt,
			    req->temperature,
			    req->fields_updated);
>>>>>>> 2d5d64565cfcb504db61fd9bba431d0afd875584
        }
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HilSensorPlugin, mavros::plugin::PluginBase)

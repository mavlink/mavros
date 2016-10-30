/**
 * @brief Mixin for hil_sensor plugins
 * @file hil_sensors_mixin.h
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

#pragma once

#include <functional>
#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>


namespace mavros {
namespace plugin {
/**
 * @brief This mixin adds set_hil_sensor()
 */
template <class D>
class SetHilSensorMixin {
public:
	//! Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_SENSOR
	void set_hil_sensor(uint64_t time_boot_us,
                        float xacc, float yacc, float zacc,
                        float xgyro, float ygyro, float zgyro,
                        float xmag, float ymag, float zmag,
                        float abs_pressure, float diff_pressure, float pressure_alt,
                        float temperature,
                        uint32_t fields_updated)
	{
		mavros::UAS *m_uas_ = static_cast<D *>(this)->m_uas;
		mavlink::common::msg::HIL_SENSOR sensor;

        // there is no target sys in this mavlink message!
		//m_uas_->msg_set_target(sensor);

        sensor.time_usec= time_boot_us;
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

		UAS_FCU(m_uas_)->send_message_ignore_drop(sensor);
	}
};

/**
 * @brief This mixin adds set_hil_state_quaternion()
 */
template <class D>
class SetHilStateQuaternionMixin {
public:
    //! Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_STATE_QUATERNION
    void set_hil_state_quaternion(uint64_t time_boot_us,
                                  float qw, float qx, float qy, float qz,
                                  float rollspeed, float pitchspeed, float yawspeed,
                                  int32_t lat, int32_t lon, int32_t alt,
                                  int16_t vx, int16_t vy, int16_t vz,
                                  uint16_t ind_airspeed, uint16_t true_airspeed,
                                  int16_t xacc, int16_t yacc, int16_t zacc)
    {
        mavros::UAS *m_uas_ = static_cast<D *>(this)->m_uas;
        mavlink::common::msg::HIL_STATE_QUATERNION state_quat;
        
        // there is no target sys in this mavlink message!
        //m_uas_->msg_set_target(sensor);
        
        state_quat.time_usec= time_boot_us;
        
        state_quat.attitude_quaternion[0]=qw;
        state_quat.attitude_quaternion[1]=qx;
        state_quat.attitude_quaternion[2]=qy;
        state_quat.attitude_quaternion[3]=qz;
        
        state_quat.rollspeed=rollspeed;
        state_quat.pitchspeed=pitchspeed;
        state_quat.yawspeed=yawspeed;
        
        state_quat.lat=lat;
        state_quat.lon=lon;
        state_quat.alt=alt;
        
        state_quat.vx=vx;
        state_quat.vy=vy;
        state_quat.vz=vz;
        
        state_quat.ind_airspeed=ind_airspeed;
        state_quat.true_airspeed=true_airspeed;
        
        state_quat.xacc=xacc;
        state_quat.yacc=yacc;
        state_quat.zacc=zacc;
        
        UAS_FCU(m_uas_)->send_message_ignore_drop(state_quat);
    }
};

}	// namespace plugin
}	// namespace mavros

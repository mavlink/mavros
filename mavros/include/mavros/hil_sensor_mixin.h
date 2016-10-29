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
			Eigen::Vector3d acc,
			Eigen::Vector3d gyro,
			Eigen::Vector3d mag,
            Eigen::Vector3d pressure,
            float temperature,
			uint32_t fields_updated)
	{
		mavros::UAS *m_uas_ = static_cast<D *>(this)->m_uas;
		mavlink::common::msg::HIL_SENSOR sensor;

        // there is no target sys in this mavlink message!
		//m_uas_->msg_set_target(sensor);

        sensor.time_usec= time_boot_us;
        sensor.xacc=acc.x();
        sensor.yacc=acc.y();
        sensor.zacc=acc.z();
        sensor.xgyro=gyro.x();
        sensor.ygyro=gyro.y();
        sensor.zgyro=gyro.z();
        sensor.xmag=mag.x();
        sensor.ymag=mag.y();
        sensor.zmag=mag.z();
        sensor.abs_pressure=pressure.x();
        sensor.diff_pressure=pressure.y();
        sensor.pressure_alt=pressure.z();
        sensor.temperature=temperature;
        sensor.fields_updated=fields_updated;

		UAS_FCU(m_uas_)->send_message_ignore_drop(sensor);
	}
};

/**
 * @brief This mixin adds set_hil_state_quaternion()
 */
template <class D>
class SetHilSensorMixin {
public:
    //! Message specification: @p https://pixhawk.ethz.ch/mavlink/#HIL_STATE_QUATERNION
    void set_hil_state_quaternion(uint64_t time_boot_us,
                        Eigen::Vector4d quat,
                        Eigen::Vector3d angularspeed,
                        Eigen::Vector3d GPS,
                        Eigen::Vector3d groundspeed,
                        uint16_t ind_airspeed,
                        uint16_t true_airspeed,
                        Eigen::Vector3d acceleration)
    {
        mavros::UAS *m_uas_ = static_cast<D *>(this)->m_uas;
        mavlink::common::msg::HIL_STATE_QUATERNION state_quat;
        
        // there is no target sys in this mavlink message!
        //m_uas_->msg_set_target(sensor);
        
        state_quat.time_usec= time_boot_us;
        
        state_quat.attitude_quaternion[0]=quat(0);
        state_quat.attitude_quaternion[1]=quat(1);
        state_quat.attitude_quaternion[2]=quat(2);
        state_quat.attitude_quaternion[3]=quat(3);
        
        state_quat.rollspeed=angularspeed.x();
        state_quat.pitchspeed=angularspeed.y();
        state_quat.yawspeed=angularspeed.z();
        
        state_quat.lat=GPS.x();
        state_quat.lon=GPS.y();
        state_quat.alt=GPS.z();
        
        state_quat.vx=groundspeed.x();
        state_quat.vy=groundspeed.y();
        state_quat.vz=groundspeed.z();
        
        state_quat.ind_airspeed=ind_airspeed;
        state_quat.true_airspeed=true_airspeed;
        
        state_quat.xacc=acceleration.x();
        state_quat.yacc=acceleration.y();
        state_quat.zacc=acceleration.z();
        
        UAS_FCU(m_uas_)->send_message_ignore_drop(state_quat);
    }
};

}	// namespace plugin
}	// namespace mavros

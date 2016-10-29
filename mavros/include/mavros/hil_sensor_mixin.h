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
            float temperature
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

}	// namespace plugin
}	// namespace mavros

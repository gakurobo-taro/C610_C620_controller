/*
 * board_info.hpp
 *
 *  Created on: 2024/01/06
 *      Author: yaa3k
 */

#ifndef BOARD_INFO_HPP_
#define BOARD_INFO_HPP_

#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

namespace G24_STM32HAL::RmcBoard{
	enum class CommonReg:uint16_t{
		ID_REQEST,
		ID_RESPONSE,
		EMERGENCY_STOP = 0x0E,
		RESET_EMERGENCY_STOP = 0x0F,
	};

	enum class RmcReg:uint16_t{
		NOP,
		MOTOR_TYPE,
		CONTROL_TYPE,
		GEAR_RATIO,
		MOTOR_STATE,

		PWM = 0x10,
		PWM_TARGET,

		SPD = 0x20,
		PWM_LIM,
		SPD_TARGET,
		SPD_GAIN_P,
		SPD_GAIN_I,
		SPD_GAIN_D,

		POS = 0x30,
		SPD_LIM,
		POS_TARGET,
		POS_GAIN_P,
		POS_GAIN_I,
		POS_GAIN_D,

		MONITOR_PERIOD = 0x40,
		MONITOR_REG1,
		MONITOR_REG2,
		MONITOR_REG3,
		MONITOR_REG4,
	};
}


#endif /* BOARD_INFO_HPP_ */

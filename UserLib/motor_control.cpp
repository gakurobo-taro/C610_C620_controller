/*
 * c610_c620_control.cpp
 *
 *  Created on: 2024/01/02
 *      Author: yaa3k
 */

#include "motor_control.hpp"

namespace G24_STM32HAL::RmcLib{

float MotorDriver::update_operation_val(const MotorState &_state){
	state = _state;
	switch(mode){
	case ControlMode::PWM_MODE:
		//nop
		break;
	case ControlMode::SPEED_MODE:
		pwm = speed_pid(target_speed,state.speed);
		break;
	case ControlMode::POSITION_MODE:
		target_speed = position_pid(target_rad,state.rad);
		pwm = speed_pid(target_speed,state.speed);
		break;
	case ControlMode::ABS_POSITION_MODE:
		//TODO:実装
		break;
	default:
		//nop
		break;
	}
	return pwm;
}

}

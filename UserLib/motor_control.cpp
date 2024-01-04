/*
 * c610_c620_control.cpp
 *
 *  Created on: 2024/01/02
 *      Author: yaa3k
 */

#include "motor_control.hpp"

namespace G24_STM32HAL::RmcLib{

void C610Driver::calc(const C6x0State state){

	speed = (float)state.speed * ks;
	rad = enc.update_angle(state.angle);


	switch(mode){
	case ControlMode::PWM_MODE:
		//nop
		break;
	case ControlMode::SPEED_MODE:
		pwm = speed_pid(target_speed,speed);
		break;
	case ControlMode::POSITION_MODE:
		target_speed = position_pid(target_rad,rad);
		pwm = speed_pid(target_speed,speed);
		break;
	case ControlMode::ABS_POSITION_MODE:
		//TODO:実装
		break;
	default:
		//nop
		break;
	}
}

}

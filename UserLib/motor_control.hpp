/*
 * c610_c620_control.hpp
 *
 *  Created on: 2023/12/28
 *      Author: yaa3k
 */

#ifndef MOTOR_CONTROL_HPP_
#define MOTOR_CONTROL_HPP_

#include <algorithm>
#include <cmath>
#include "main.h"
#include "pid.hpp"
#include "encoder.hpp"
#include "STM32HAL_CommonLib/can_comm.hpp"

namespace G24_STM32HAL::RmcLib{

	enum class ControlMode{
		PWM_MODE,
		SPEED_MODE,
		POSITION_MODE,
		ABS_POSITION_MODE,
	};

	struct MotorState{
		float rad;
		float speed;
		float current;
		float temperature;
	};

	struct C6x0State:MotorState{
	private:
		const float gear_ratio;
		const float ks;
	public:
		AngleEncoder encoder;
		C6x0State(float _gear_ratio):
			gear_ratio(_gear_ratio),ks(2*M_PI/(gear_ratio*360.0f)),
			encoder(gear_ratio,13){}

		bool update(CommonLib::CanFrame frame){
			if(frame.is_ext_id || frame.is_remote || frame.data_length != 8 || !(0x200&frame.id)){
				return false;
			}
			rad = encoder.update_angle(frame.data[0]<<8 | frame.data[1]);
			speed = (float)(int16_t)(frame.data[2]<<8 | frame.data[3]) * ks;
			current = (float)(frame.data[4]<<8 | frame.data[5]);
			temperature = (float)frame.data[6];
			return true;
		}
	};

	class MotorDriver{
	private:
		ControlMode mode = ControlMode::PWM_MODE;
		float pwm;
		float target_speed;
		float target_rad;
		MotorState state;

		PID speed_pid = PID{1,-1,1};
		PID position_pid = PID{1,-260,260};

	public:
		//mode setting
		void set_control_mode(ControlMode _mode){ mode = _mode; }
		ControlMode get_control_mode(void){ return mode; }

		//pwm control
		void set_pwm(float _pwm){ pwm = std::clamp(_pwm,-1.0f,1.0f); }
		float get_pwm(void){ return pwm; }

		//speed control
		void set_speed_gain(float kp,float ki,float kd){speed_pid.set_gain(kp, ki, kd);}
		void set_target_speed(float rad_per_sec){ target_speed = rad_per_sec; }
		float get_target_speed(void){return target_speed; }
		float get_current_speed(void){return state.speed;}

		//position control
		void set_position_gain(float kp,float ki,float kd){position_pid.set_gain(kp, ki, kd);}
		void set_target_position(float rad){target_rad = rad;}
		float get_target_position(void){ return target_rad; }
		float get_current_position(void){return state.rad;}

		//pid operation
		float update_operation_val(const MotorState &_state);
	};
}


#endif /* MOTOR_CONTROL_HPP_ */

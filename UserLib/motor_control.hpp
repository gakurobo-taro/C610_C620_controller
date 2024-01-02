/*
 * c610_c620_control.hpp
 *
 *  Created on: 2023/12/28
 *      Author: yaa3k
 */

#ifndef MOTOR_CONTROL_HPP_
#define MOTOR_CONTROL_HPP_

#include "main.h"
#include "pid.hpp"
#include "../STM32HAL_CommonLib/can_comm.hpp"

namespace G24_STM32HAL::RmcLib{

	enum class ControlMode{
		PWM_MODE,
		SPEED_MODE,
		POSITION_MODE,
		ABS_POSITION_MODE,
	};


	class IMotorDriver{
		virtual void set_control_mode(ControlMode mode) = 0;
		virtual ControlMode get_control_mode(void) = 0;

		virtual bool set_pwm(float  duty) = 0;
		virtual bool get_pwm(float duty) = 0;

		virtual bool set_target_speed(float rad_per_sec) = 0;
		virtual float get_target_speed(void) = 0;
		virtual float get_current_speed(void) = 0;

		virtual bool set_target_positon(float rad) = 0;
		virtual float get_target_positon(void) = 0;
		virtual float get_current_positon(void) = 0;

		virtual bool is_active(void) = 0;
	};

	struct C6x0State{
		uint32_t id;
		uint16_t angle; //0~8191
		int16_t speed;  //rpm
		int16_t motor_current;
		uint8_t motor_temperature; //only C620

		bool convert_from_can_frame(CommonLib::CanFrame frame){
			if(frame.is_ext_id || frame.is_remote || frame.data_length != 8){
				return false;
			}
			auto reader = frame.reader();
			id = frame.id;
			angle = reader.read<uint16_t>().value();
			speed = reader.read<int16_t>().value();
			motor_current = reader.read<uint8_t>().value();
		}
	};

//	class C610Driver:IMotorDriver{
//	private:
//		ControlMode mode = ControlMode::PWM_MODE;
//		float target_pwm;
//		float pwm;
//		float target_speed;
//		float speed;
//		float target_position;
//		float position;
//
//		PID speed_pid = PID{1,-1,1};
//		PID position_pid = PID{};
//
//	public:
//
//		void set_control_mode(ControlMode _mode) override{ mode = _mode; }
//		ControlMode get_control_mode(void) override{ return mode; }
//
//		bool set_pwm(float duty)override{ pwm = std::clamp(duty,-1,1); }
//
//		float get_pwm(void)override{ return pwm; }
//
//
//
//
//	};
}


#endif /* MOTOR_CONTROL_HPP_ */

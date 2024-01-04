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


	class IMotorDriver{
		virtual void set_control_mode(ControlMode mode) = 0;
		virtual ControlMode get_control_mode(void) = 0;

		virtual void set_pwm(float  _pwm) = 0;
		virtual float get_pwm(void) = 0;

		virtual void set_target_speed(float rad_per_sec) = 0;
		virtual float get_target_speed(void) = 0;
		virtual float get_current_speed(void) = 0;

		virtual void set_target_position(float rad) = 0;
		virtual float get_target_position(void) = 0;
		virtual float get_current_position(void) = 0;

		virtual void set_speed_gain(float kp,float ki,float kd) = 0;
		virtual void set_position_gain(float kp,float ki,float kd) = 0;

		virtual bool is_active(void) = 0;
	};

	struct C6x0State{
		uint32_t id;
		uint16_t angle; //0~8191
		int16_t speed;  //rpm
		int16_t current;
		uint8_t temperature; //only C620

		bool convert_from_can_frame(CommonLib::CanFrame frame){
			if(frame.is_ext_id || frame.is_remote || frame.data_length != 8){
				return false;
			}
			id = frame.id;
			angle = frame.data[0]<<8 | frame.data[1];
			speed = frame.data[2]<<8 | frame.data[3];
			current = frame.data[4]<<8 | frame.data[5];
			temperature = frame.data[6];
			//TODO:エディアン考慮したreader/writer
			return true;
		}
	};

	class C610Driver:IMotorDriver{
	private:
		static constexpr float ks = 2*M_PI/(36.0f*360.0f);

		ControlMode mode = ControlMode::PWM_MODE;
		float pwm;
		float target_speed;
		float speed;
		float target_rad;
		float rad;

		C6x0Encoder enc = C6x0Encoder{36.0f,13};

		PID speed_pid = PID{1,-1,1};
		PID position_pid = PID{1,-260,260};

	public:
		void set_control_mode(ControlMode _mode) override{ mode = _mode; }
		ControlMode get_control_mode(void) override{ return mode; }

		void set_pwm(float _pwm)override{ pwm = std::clamp(_pwm,-1.0f,1.0f); }
		float get_pwm(void)override{ return pwm; }

		void set_target_speed(float rad_per_sec)override{ target_speed = rad_per_sec; }

		float get_target_speed(void) override{return target_speed; }
		float get_current_speed(void) override{return speed;}

		void set_target_position(float rad) override {target_rad = rad;}
		float get_target_position(void)override{ return target_rad; }
		float get_current_position(void)override{return rad;}

		void set_speed_gain(float kp,float ki,float kd)override{speed_pid.set_gain(kp, ki, kd);}
		void set_position_gain(float kp,float ki,float kd)override{position_pid.set_gain(kp, ki, kd);}

		bool is_active(void)override{return true;}

		void calc(const C6x0State state);

	};
}


#endif /* MOTOR_CONTROL_HPP_ */

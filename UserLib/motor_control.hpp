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

	enum class ControlMode:uint8_t{
		PWM_MODE,
		SPEED_MODE,
		POSITION_MODE,
		ABS_POSITION_MODE,
	};

	struct MotorState{
		float rad;
		float speed;
	};

	//ロボマス内臓エンコーダ
	struct C6x0State:MotorState{
	private:
		float gear_ratio;
		float gear_ratio_inv;
		float ks;
		AngleEncoder encoder;
	public:
		float current;
		float temperature;
		C6x0State(float _gear_ratio = 1):
			gear_ratio(_gear_ratio),gear_ratio_inv(1/_gear_ratio),
			ks(2*M_PI/(gear_ratio*60.0f)),encoder(13){}

		bool update(CommonLib::CanFrame frame){
			if(frame.is_ext_id || frame.is_remote || frame.data_length != 8 || !(0x200&frame.id)){
				return false;
			}
			uint16_t angle = frame.data[0]<<8 | frame.data[1];
			int16_t angle_speed = frame.data[2]<<8 | frame.data[3];

			rad = encoder.update_angle(angle,angle_speed)*gear_ratio_inv;
			speed = (float)angle_speed * ks;
			current = (float)(frame.data[4]<<8 | frame.data[5]);
			temperature = (float)frame.data[6];
			return true;
		}
		void set_gear_ratio(float ratio){
			gear_ratio = ratio;
			gear_ratio_inv = 1/ratio;
			ks = 2*M_PI/(gear_ratio*60.0f);
		}
		float get_gear_ratio(void)const{
			return gear_ratio;
		}
	};

	//AS5600による制御
	struct AS5600State:MotorState{
	private:
		static constexpr uint16_t as5600_id = 0x36;
		static constexpr size_t as5600_resolution = 12;
		static constexpr float ks = 2*M_PI/(float)((1<<as5600_resolution)-1);

		I2C_HandleTypeDef* i2c;
		AngleEncoder encoder;
		const float freq;

		uint16_t enc_val = 0;
		float rad_old = 0;

		bool inv = false;
	public:
		AS5600State(I2C_HandleTypeDef* _i2c,float _freq):i2c(_i2c),encoder(as5600_resolution),freq(_freq){
		}

		void read_start(void){
			HAL_I2C_Master_Receive_IT(i2c, as5600_id<<1, (uint8_t*)&enc_val, 2);
		}
		void i2c_rx_interrupt_task(void){
			rad = encoder.update_angle(enc_val)*(inv?-1.0f:1.0f);
			speed = (rad - rad_old)*freq*(inv?-1.0f:1.0f);
			rad_old = rad;
		}
	};

	class MotorDriver{
	private:
		ControlMode mode = ControlMode::PWM_MODE;
		float pwm;
		float target_speed;
		float target_rad;
		float origin;
		MotorState state;
		MotorState abs_state;

		PID speed_pid = PIDBuilder(1000.0f).set_limit(-1.0f,1.0f).build();
		PID position_pid = PIDBuilder(1000.0f).set_limit(-7.0f,7.0f).build();
	public:
		//mode setting
		void set_control_mode(ControlMode _mode);
		ControlMode get_control_mode(void){ return mode; }
		void set_origin(float origin_rad){origin = origin_rad;}

		//pwm control
		void set_pwm(float _pwm){ pwm = std::clamp(_pwm,-1.0f,1.0f); }
		float get_pwm(void){ return pwm; }

		//speed control
		void set_speed_gain(const PIDGain &gain){speed_pid.set_gain(gain);}
		void set_pwm_limit(float min,float max){speed_pid.set_limit(min, max);}
		void set_pwm_limit(float max){speed_pid.set_limit(-max, max);}
		void set_target_speed(float rad_per_sec){ target_speed = rad_per_sec; }
		float get_target_speed(void)const{return target_speed; }
		float get_current_speed(void)const{return state.speed;}
		PIDGain get_speed_gain(void)const{return speed_pid.get_gain();}

		//position control
		void set_position_gain(const PIDGain &gain){position_pid.set_gain(gain);}
		void set_speed_limit(float min,float max){position_pid.set_limit(min, max);}
		void set_speed_limit(float max){position_pid.set_limit(-max, max);}
		void set_target_position(float rad){target_rad = rad + origin;}
		void set_target_low_position(float rad){target_rad = rad ;}
		float get_target_position(void)const{ return target_rad - origin; }
		float get_current_position(void)const{return state.rad - origin;}
		float get_current_low_position(void)const{return state.rad; }
		PIDGain get_position_gain(void)const{return position_pid.get_gain();}

		//abs position
		float get_abs_position(void){return abs_state.rad;};
		float get_abs_speed(void){return abs_state.speed;};

		//pid operation
		float update_operation_val(const MotorState &_state,const MotorState &_abs_state);
	};
}


#endif /* MOTOR_CONTROL_HPP_ */

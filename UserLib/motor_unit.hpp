/*
 * motor_unit.hpp
 *
 *  Created on: Mar 5, 2024
 *      Author: gomas
 */

#ifndef MOTOR_UNIT_HPP_
#define MOTOR_UNIT_HPP_

#include "board_info.hpp"
#include "motor_control.hpp"
#include "STM32HAL_CommonLib/id_map_control.hpp"

namespace G24_STM32HAL::RmcBoard{
	inline auto set_p_gain = [](RmcLib::PIDGain g,float p)mutable->RmcLib::PIDGain {g.kp = p; return g;};
	inline auto set_i_gain = [](RmcLib::PIDGain g,float i)mutable->RmcLib::PIDGain {g.kp = i; return g;};
	inline auto set_d_gain = [](RmcLib::PIDGain g,float d)mutable->RmcLib::PIDGain {g.kp = d; return g;};

	struct MotorUnit{
		RmcLib::MotorDriver driver;
		RmcLib::C6x0State motor_enc;
		RmcLib::AS5600State abs_enc;
		std::bitset<0x35+1> monitor;
		CommonLib::IDMap id_map;

		MotorUnit(RmcLib::MotorDriver &&_driver,RmcLib::C6x0State &&_motor_enc,RmcLib::AS5600State &&_abs_enc,std::bitset<0x35+1> &&_monitor,CommonLib::IDMap &&_id_map)
			:driver(_driver),motor_enc(_motor_enc),abs_enc(_abs_enc),monitor(_monitor),id_map(_id_map){}
	};

	class MotorUnitBuilder{
		RmcLib::MotorDriver d;
		RmcLib::C6x0State enc;
		RmcLib::AS5600State abs_enc;
		std::bitset<0x35+1> monitor;
		CommonLib::IDMap id_map;

		MotorUnitBuilder& driver(void){d = RmcLib::MotorDriver(); return *this;}
		MotorUnitBuilder& motor_encoder(float r){enc = RmcLib::AS5600State(r); return *this;}
		MotorUnitBuilder& abs_encoder(I2C_HandleTypeDef* _i2c,float _freq,GPIO_TypeDef *_port,uint16_t _pin){
			abs_enc = RmcLib::AS5600State(_i2c,_freq,_port,_pin);
			return *this;
		}
		MotorUnitBuilder& monitor(void){monitor = std::bitset<0x35+1>(); return *this;}
		MotorUnitBuilder& id_map(void){
			id_map = CommonLib::IDMapBuilder()
					.add((uint16_t)RmcReg::CONTROL_TYPE,  CommonLib::DataAccessor::generate<RmcLib::ControlMode>([](RmcLib::ControlMode m){driver[0].set_control_mode(m);},[]()->RmcLib::ControlMode{return driver[0].get_control_mode();}))
					.add((uint16_t)RmcReg::GEAR_RATIO,    CommonLib::DataAccessor::generate<float>([](float ratio){motor_state[0].set_gear_ratio(ratio);},[]()->float{return motor_state[0].get_gear_ratio();}))
					.add((uint16_t)RmcReg::CAN_TIMEOUT,   CommonLib::DataAccessor::generate<uint16_t>([](uint16_t period){timeout_en_flag = false; set_timer_period(can_timeout_timer,period);},[]()->uint16_t{return get_timer_period(can_timeout_timer);}))

					.add((uint16_t)RmcReg::PWM,           CommonLib::DataAccessor::generate<float>([]()->float{return driver[0].get_pwm();}))
					.add((uint16_t)RmcReg::PWM_TARGET,    CommonLib::DataAccessor::generate<float>([](float pwm){driver[0].set_pwm(pwm);},[]()->float{return driver[0].get_pwm();}))

					.add((uint16_t)RmcReg::SPD,           CommonLib::DataAccessor::generate<float>([]()->float{return driver[0].get_current_speed();}))
					.add((uint16_t)RmcReg::SPD_TARGET,    CommonLib::DataAccessor::generate<float>([](float st){driver[0].set_target_speed(st);},[]()->float{return driver[0].get_target_speed();}))
					.add((uint16_t)RmcReg::PWM_LIM,       CommonLib::DataAccessor::generate<float>([](float pl){driver[0].set_pwm_limit(pl);}))
					.add((uint16_t)RmcReg::SPD_GAIN_P,    CommonLib::DataAccessor::generate<float>([](float sp){driver[0].set_speed_gain(set_p_gain(driver[0].get_speed_gain(),sp));},[]()->float{return driver[0].get_speed_gain().kp;}))
					.add((uint16_t)RmcReg::SPD_GAIN_I,    CommonLib::DataAccessor::generate<float>([](float si){driver[0].set_speed_gain(set_i_gain(driver[0].get_speed_gain(),si));},[]()->float{return driver[0].get_speed_gain().ki;}))
					.add((uint16_t)RmcReg::SPD_GAIN_D,    CommonLib::DataAccessor::generate<float>([](float sd){driver[0].set_speed_gain(set_d_gain(driver[0].get_speed_gain(),sd));},[]()->float{return driver[0].get_speed_gain().kd;}))

					.add((uint16_t)RmcReg::POS,           CommonLib::DataAccessor::generate<float>([](float pos){driver[0].set_origin(driver[0].get_current_low_position()-pos);},[]()->float{return driver[0].get_current_position();}))
					.add((uint16_t)RmcReg::POS_TARGET,    CommonLib::DataAccessor::generate<float>([](float pt){driver[0].set_target_position(pt);},[]()->float{return driver[0].get_target_position();}))
					.add((uint16_t)RmcReg::SPD_LIM,       CommonLib::DataAccessor::generate<float>([](float sl){driver[0].set_speed_limit(sl);}))
					.add((uint16_t)RmcReg::POS_GAIN_P,    CommonLib::DataAccessor::generate<float>([](float pp){driver[0].set_position_gain(set_p_gain(driver[0].get_position_gain(),pp));},[]()->float{return driver[0].get_position_gain().kp;}))
					.add((uint16_t)RmcReg::POS_GAIN_I,    CommonLib::DataAccessor::generate<float>([](float pi){driver[0].set_position_gain(set_i_gain(driver[0].get_position_gain(),pi));},[]()->float{return driver[0].get_position_gain().ki;}))
					.add((uint16_t)RmcReg::POS_GAIN_D,    CommonLib::DataAccessor::generate<float>([](float pd){driver[0].set_position_gain(set_d_gain(driver[0].get_position_gain(),pd));},[]()->float{return driver[0].get_position_gain().kd;}))

					.add((uint16_t)RmcReg::MONITOR_PERIOD,CommonLib::DataAccessor::generate<uint16_t>([](uint16_t period){set_timer_period(monitor_timer,period);}, []()->uint16_t{return get_timer_period(monitor_timer);}))
					.add((uint16_t)RmcReg::MONITOR_REG,   CommonLib::DataAccessor::generate<uint64_t>([](uint64_t val){ monitor[0] = std::bitset<0x35+1>{val};}, []()->uint64_t{ return monitor[0].to_ullong();}))
					.build();
			return *this;
		}

		MotorUnit build(void){
			return MotorUnit(std::move(d),std::move(enc),std::move(abs_enc),std::move(monitor),std::move(id_map));
		}
	};
}



#endif /* MOTOR_UNIT_HPP_ */

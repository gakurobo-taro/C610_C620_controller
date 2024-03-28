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

#include "STM32HAL_CommonLib/timer_control.hpp"
#include "STM32HAL_CommonLib/LED_control.hpp"
#include "STM32HAL_CommonLib/data_packet.hpp"
#include "STM32HAL_CommonLib/id_map_control.hpp"

#include <bitset>

namespace G24_STM32HAL::RmcBoard{

	struct MotorUnit{
		CommonLib::LEDHALGpio led;
		RmcLib::MotorDriver driver;
		RmcLib::C6x0State motor_enc;
		RmcLib::AS5600State abs_enc;
		std::bitset<64> monitor;

		RmcLib::ControlMode mode_tmp = RmcLib::ControlMode::PWM_MODE;

		MotorType motor_type = MotorType::C6x0;

		MotorUnit(GPIO_TypeDef *led_port,uint16_t led_pin,float gear_ratio,I2C_HandleTypeDef *i2c,float freq,GPIO_TypeDef *i2c_sel_port,uint16_t i2c_sel_pin)
			:led(led_port,led_pin),
			 motor_enc(gear_ratio),
			 monitor(),
			 abs_enc(i2c,freq,i2c_sel_port,i2c_sel_pin),
			 driver(){
			 }
	};
	class MotorUnitBuilder{
		//LED
		GPIO_TypeDef *led_port;
		uint16_t led_pin;

		//エンコーダ
		float gear_ratio = 36.0f;
		I2C_HandleTypeDef *i2c;
		GPIO_TypeDef *i2c_sel_port;
		uint16_t i2c_sel_pin;
		float freq;

	public:
		MotorUnitBuilder& set_LED(GPIO_TypeDef *_port,uint16_t _pin){
			led_port = _port;
			led_pin = _pin;
			return *this;
		}
		MotorUnitBuilder& set_gear_ratio(float r){
			gear_ratio = r;
			return *this;
		}
		MotorUnitBuilder& set_i2c_encoder(I2C_HandleTypeDef* _i2c,float _freq,GPIO_TypeDef *_port,uint16_t _pin){
			i2c = _i2c;
			freq = _freq;
			i2c_sel_port = _port;
			i2c_sel_pin = _pin;
			return *this;
		}

		MotorUnit build(void){
			return std::move(MotorUnit(led_port,led_pin,gear_ratio,i2c,freq,i2c_sel_port,i2c_sel_pin));
		}
	};

	inline CommonLib::IDMap map_build(MotorUnit &unit,CommonLib::IInterruptionTimer &timeout_timer,CommonLib::IInterruptionTimer &monitor_timer){
		auto set_p_gain = [](RmcLib::PIDGain g,float p)mutable->RmcLib::PIDGain {g.kp = p; return g;};
		auto set_i_gain = [](RmcLib::PIDGain g,float i)mutable->RmcLib::PIDGain {g.ki = i; return g;};
		auto set_d_gain = [](RmcLib::PIDGain g,float d)mutable->RmcLib::PIDGain {g.kd = d; return g;};
		return CommonLib::IDMapBuilder()
				.add((uint16_t)RmcReg::MOTOR_TYPE,    CommonLib::DataAccessor::generate<MotorType>(&unit.motor_type))
				.add((uint16_t)RmcReg::CONTROL_TYPE,  CommonLib::DataAccessor::generate<RmcLib::ControlMode>([&](RmcLib::ControlMode m)mutable{unit.driver.set_control_mode(m);},[&]()mutable->RmcLib::ControlMode{return unit.driver.get_control_mode();}))
				.add((uint16_t)RmcReg::GEAR_RATIO,    CommonLib::DataAccessor::generate<float>([&](float ratio)mutable{unit.motor_enc.set_gear_ratio(ratio);},[&]()->float{return unit.motor_enc.get_gear_ratio();}))
				.add((uint16_t)RmcReg::CAN_TIMEOUT,   CommonLib::DataAccessor::generate<uint16_t>([&](uint16_t period)mutable{timeout_timer.set_and_start(period);},[&]()->uint16_t{return timeout_timer.get_state();}))

				.add((uint16_t)RmcReg::PWM,           CommonLib::DataAccessor::generate<float>([&]()->float{return unit.driver.get_pwm();}))
				.add((uint16_t)RmcReg::PWM_TARGET,    CommonLib::DataAccessor::generate<float>([&](float pwm)mutable{unit.driver.set_pwm(pwm);},[&]()->float{return unit.driver.get_pwm();}))

				.add((uint16_t)RmcReg::SPD,           CommonLib::DataAccessor::generate<float>([&]()->float{return unit.driver.get_current_speed();}))
				.add((uint16_t)RmcReg::SPD_TARGET,    CommonLib::DataAccessor::generate<float>([&](float st)mutable{unit.driver.set_target_speed(st);},[&]()->float{return unit.driver.get_target_speed();}))
				.add((uint16_t)RmcReg::PWM_LIM,       CommonLib::DataAccessor::generate<float>([&](float pl)mutable{unit.driver.set_pwm_limit(pl);}))
				.add((uint16_t)RmcReg::SPD_GAIN_P,    CommonLib::DataAccessor::generate<float>([&](float sp)mutable{unit.driver.set_speed_gain(set_p_gain(unit.driver.get_speed_gain(),sp));},[&]()->float{return unit.driver.get_speed_gain().kp;}))
				.add((uint16_t)RmcReg::SPD_GAIN_I,    CommonLib::DataAccessor::generate<float>([&](float si)mutable{unit.driver.set_speed_gain(set_i_gain(unit.driver.get_speed_gain(),si));},[&]()->float{return unit.driver.get_speed_gain().ki;}))
				.add((uint16_t)RmcReg::SPD_GAIN_D,    CommonLib::DataAccessor::generate<float>([&](float sd)mutable{unit.driver.set_speed_gain(set_d_gain(unit.driver.get_speed_gain(),sd));},[&]()->float{return unit.driver.get_speed_gain().kd;}))

				.add((uint16_t)RmcReg::POS,           CommonLib::DataAccessor::generate<float>([&](float pos)mutable{unit.driver.set_origin(unit.driver.get_current_low_position()-pos);},[&]()->float{return unit.driver.get_current_position();}))
				.add((uint16_t)RmcReg::POS_TARGET,    CommonLib::DataAccessor::generate<float>([&](float pt)mutable{unit.driver.set_target_position(pt);},[&]()->float{return unit.driver.get_target_position();}))
				.add((uint16_t)RmcReg::SPD_LIM,       CommonLib::DataAccessor::generate<float>([&](float sl)mutable{unit.driver.set_speed_limit(sl);}))
				.add((uint16_t)RmcReg::POS_GAIN_P,    CommonLib::DataAccessor::generate<float>([&](float pp)mutable{unit.driver.set_position_gain(set_p_gain(unit.driver.get_position_gain(),pp));},[&]()->float{return unit.driver.get_position_gain().kp;}))
				.add((uint16_t)RmcReg::POS_GAIN_I,    CommonLib::DataAccessor::generate<float>([&](float pi)mutable{unit.driver.set_position_gain(set_i_gain(unit.driver.get_position_gain(),pi));},[&]()->float{return unit.driver.get_position_gain().ki;}))
				.add((uint16_t)RmcReg::POS_GAIN_D,    CommonLib::DataAccessor::generate<float>([&](float pd)mutable{unit.driver.set_position_gain(set_d_gain(unit.driver.get_position_gain(),pd));},[&]()->float{return unit.driver.get_position_gain().kd;}))
				.add((uint16_t)RmcReg::ABS_POS,       CommonLib::DataAccessor::generate<float>([&]()->float{return unit.driver.get_abs_position();}))
				.add((uint16_t)RmcReg::ABS_SPD,       CommonLib::DataAccessor::generate<float>([&]()->float{return unit.driver.get_abs_speed();}))
				.add((uint16_t)RmcReg::ENC_INV,       CommonLib::DataAccessor::generate<bool>([&](bool inv)mutable{unit.abs_enc.set_enc_inv(inv);},[&]()->bool{return unit.abs_enc.is_inv();}))
				.add((uint16_t)RmcReg::ABS_TURN_CNT,  CommonLib::DataAccessor::generate<int32_t>([&](int32_t cnt)mutable{unit.abs_enc.set_turn_count(cnt);},[&]()->int32_t{return unit.abs_enc.get_turn_count();}))

				.add((uint16_t)RmcReg::MONITOR_PERIOD,CommonLib::DataAccessor::generate<uint16_t>([&](uint16_t period)mutable{monitor_timer.set_and_start(period);}, [&]()->uint16_t{return monitor_timer.get_state();}))
				.add((uint16_t)RmcReg::MONITOR_REG,   CommonLib::DataAccessor::generate<uint64_t>([&](uint64_t val)mutable{ unit.monitor = std::bitset<64>{val};}, [&]()->uint64_t{ return unit.monitor.to_ullong();}))
				.build();
	}

}



#endif /* MOTOR_UNIT_HPP_ */

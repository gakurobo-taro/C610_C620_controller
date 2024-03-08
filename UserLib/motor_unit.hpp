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
#include "LED_control.hpp"
#include "timer_control.hpp"
#include "STM32HAL_CommonLib/data_packet.hpp"
#include "STM32HAL_CommonLib/id_map_control.hpp"

#include <bitset>

namespace G24_STM32HAL::RmcBoard{
	inline auto set_p_gain = [](RmcLib::PIDGain g,float p)mutable->RmcLib::PIDGain {g.kp = p; return g;};
	inline auto set_i_gain = [](RmcLib::PIDGain g,float i)mutable->RmcLib::PIDGain {g.kp = i; return g;};
	inline auto set_d_gain = [](RmcLib::PIDGain g,float d)mutable->RmcLib::PIDGain {g.kp = d; return g;};

	struct MotorUnit{
		RmcLib::LEDGPIO led;
		RmcLib::MotorDriver driver;
		RmcLib::C6x0State motor_enc;
		RmcLib::AS5600State abs_enc;
		std::bitset<0x35+1> monitor;
		CommonLib::IDMap id_map;

		RmcLib::ControlMode mode_tmp;

		std::function<void(RmcLib::ControlMode)> control_set = [this](RmcLib::ControlMode m)mutable{driver.set_control_mode(m);};
		std::function<RmcLib::ControlMode(void)> control_get = [this]()mutable->RmcLib::ControlMode{return driver.get_control_mode();};

		MotorUnit(GPIO_TypeDef *led_port,uint16_t led_pin,float gear_ratio,I2C_HandleTypeDef *i2c,GPIO_TypeDef *i2c_sel_port,uint16_t i2c_sel_pin,float freq,RmcLib::IInterruptionTimer *timeout_timer,RmcLib::IInterruptionTimer *monitor_timer)
			:led(led_port,led_pin),
			 driver(),
			 motor_enc(gear_ratio),
			 abs_enc(i2c,freq,i2c_sel_port,i2c_sel_pin),
			 monitor(),
			 id_map(CommonLib::IDMapBuilder()
					.add((uint16_t)RmcReg::CONTROL_TYPE,  CommonLib::DataAccessor::generate<RmcLib::ControlMode>(std::move(control_set),std::move(control_get)))
					.add((uint16_t)RmcReg::GEAR_RATIO,    CommonLib::DataAccessor::generate<float>([this](float ratio)mutable{motor_enc.set_gear_ratio(ratio);},[this]()->float{return motor_enc.get_gear_ratio();}))
					.add((uint16_t)RmcReg::CAN_TIMEOUT,   CommonLib::DataAccessor::generate<uint16_t>([=](uint16_t period)mutable{timeout_timer->set_and_start(period);},[=]()->uint16_t{return timeout_timer->get_state();}))

					.add((uint16_t)RmcReg::PWM,           CommonLib::DataAccessor::generate<float>([this]()->float{return driver.get_pwm();}))
					.add((uint16_t)RmcReg::PWM_TARGET,    CommonLib::DataAccessor::generate<float>([this](float pwm)mutable{driver.set_pwm(pwm);},[this]()->float{return driver.get_pwm();}))

					.add((uint16_t)RmcReg::SPD,           CommonLib::DataAccessor::generate<float>([this]()->float{return driver.get_current_speed();}))
					.add((uint16_t)RmcReg::SPD_TARGET,    CommonLib::DataAccessor::generate<float>([this](float st)mutable{driver.set_target_speed(st);},[this]()->float{return driver.get_target_speed();}))
					.add((uint16_t)RmcReg::PWM_LIM,       CommonLib::DataAccessor::generate<float>([this](float pl)mutable{driver.set_pwm_limit(pl);}))
					.add((uint16_t)RmcReg::SPD_GAIN_P,    CommonLib::DataAccessor::generate<float>([this](float sp)mutable{driver.set_speed_gain(set_p_gain(driver.get_speed_gain(),sp));},[this]()->float{return driver.get_speed_gain().kp;}))
					.add((uint16_t)RmcReg::SPD_GAIN_I,    CommonLib::DataAccessor::generate<float>([this](float si)mutable{driver.set_speed_gain(set_i_gain(driver.get_speed_gain(),si));},[this]()->float{return driver.get_speed_gain().ki;}))
					.add((uint16_t)RmcReg::SPD_GAIN_D,    CommonLib::DataAccessor::generate<float>([this](float sd)mutable{driver.set_speed_gain(set_d_gain(driver.get_speed_gain(),sd));},[this]()->float{return driver.get_speed_gain().kd;}))

					.add((uint16_t)RmcReg::POS,           CommonLib::DataAccessor::generate<float>([this](float pos)mutable{driver.set_origin(driver.get_current_low_position()-pos);},[&]()->float{return driver.get_current_position();}))
					.add((uint16_t)RmcReg::POS_TARGET,    CommonLib::DataAccessor::generate<float>([this](float pt)mutable{driver.set_target_position(pt);},[&]()->float{return driver.get_target_position();}))
					.add((uint16_t)RmcReg::SPD_LIM,       CommonLib::DataAccessor::generate<float>([this](float sl)mutable{driver.set_speed_limit(sl);}))
					.add((uint16_t)RmcReg::POS_GAIN_P,    CommonLib::DataAccessor::generate<float>([this](float pp)mutable{driver.set_position_gain(set_p_gain(driver.get_position_gain(),pp));},[this]()->float{return driver.get_position_gain().kp;}))
					.add((uint16_t)RmcReg::POS_GAIN_I,    CommonLib::DataAccessor::generate<float>([this](float pi)mutable{driver.set_position_gain(set_i_gain(driver.get_position_gain(),pi));},[this]()->float{return driver.get_position_gain().ki;}))
					.add((uint16_t)RmcReg::POS_GAIN_D,    CommonLib::DataAccessor::generate<float>([this](float pd)mutable{driver.set_position_gain(set_d_gain(driver.get_position_gain(),pd));},[this]()->float{return driver.get_position_gain().kd;}))
					.add((uint16_t)RmcReg::ABS_POS,       CommonLib::DataAccessor::generate<float>([this]()->float{return driver.get_abs_position();}))
					.add((uint16_t)RmcReg::ABS_SPD,       CommonLib::DataAccessor::generate<float>([this]()->float{return driver.get_abs_speed();}))
					.add((uint16_t)RmcReg::ENC_INV,       CommonLib::DataAccessor::generate<bool>([this](bool inv)mutable{abs_enc.set_enc_inv(inv);},[this]()->bool{return abs_enc.is_inv();}))

					.add((uint16_t)RmcReg::MONITOR_PERIOD,CommonLib::DataAccessor::generate<uint16_t>([=](uint16_t period)mutable{monitor_timer->set_and_start(period);}, [=]()->uint16_t{return monitor_timer->get_state();}))
					.add((uint16_t)RmcReg::MONITOR_REG,   CommonLib::DataAccessor::generate<uint64_t>([this](uint64_t val)mutable{ monitor = std::bitset<0x35+1>{val};}, [this]()->uint64_t{ return monitor.to_ullong();}))
					.build()){}

		void test(void){
			//driver.set_control_mode(RmcLib::ControlMode::SPEED_MODE);
			auto test_a = id_map.accessors_map.find(0x2);
			if(test_a != id_map.accessors_map.end()){
				CommonLib::DataPacket test_data;
				test_data.writer().write<uint8_t>(2);
				auto reader = test_data.reader();
				//test_a->second.set(reader);
			}
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

		//タイマー
		RmcLib::IInterruptionTimer *timeout_timer;
		RmcLib::IInterruptionTimer *monitor_timer;

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
		MotorUnitBuilder& set_timeout_timer(RmcLib::IInterruptionTimer *tim){
			timeout_timer = tim;
			return *this;
		}
		MotorUnitBuilder& set_monitor_timer(RmcLib::IInterruptionTimer *tim){
			monitor_timer = tim;
			return *this;
		}

		MotorUnit build(void){
			//return MotorUnit(std::move(led),std::move(driver),std::move(motor_enc),std::move(abs_enc),std::move(mreg),timeout_timer,monitor_timer);
			return std::move(MotorUnit(led_port,led_pin,gear_ratio,i2c,i2c_sel_port,i2c_sel_pin,freq,timeout_timer,monitor_timer));
		}
	};
}



#endif /* MOTOR_UNIT_HPP_ */

/*
 * board_task.hpp
 *
 *  Created on: 2024/01/02
 *      Author: yaa3k
 */

#ifndef BOARD_TASK_HPP_
#define BOARD_TASK_HPP_

#include "board_info.hpp"

#include "motor_control.hpp"
#include "LED_control.hpp"
#include "LED_pattern.hpp"
#include "STM32HAL_CommonLib/can_comm.hpp"
#include "STM32HAL_CommonLib/pwm.hpp"
#include "STM32HAL_CommonLib/data_packet.hpp"
#include "STM32HAL_CommonLib/data_convert.hpp"
#include "STM32HAL_CommonLib/serial_comm.hpp"
#include "STM32HAL_CommonLib/id_map_control.hpp"

#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

#include <stdio.h>
#include <array>
#include <bitset>

extern USBD_HandleTypeDef hUsbDeviceFS;

namespace G24_STM32HAL::RmcBoard{
	//peripherals
	//GPIO
	struct GPIOParam{
		GPIO_TypeDef * port;
		uint16_t pin;
		GPIOParam(GPIO_TypeDef * _port,uint16_t _pin):port(_port),pin(_pin){}
	};
	inline auto dip_sw = std::array<GPIOParam,4>{
		GPIOParam{ID0_GPIO_Port,ID0_Pin},
		GPIOParam{ID1_GPIO_Port,ID1_Pin},
		GPIOParam{ID2_GPIO_Port,ID2_Pin},
		GPIOParam{ID3_GPIO_Port,ID3_Pin},
	};

	//timer
	inline auto *motor_control_timer = &htim14;
	inline auto *monitor_timer = &htim13;
	inline auto *can_timeout_timer = &htim12;

	inline auto set_timer_period = [](TIM_HandleTypeDef *tim,uint16_t val){
		if(val == 0){
			HAL_TIM_Base_Stop_IT(tim);
		}else{
			__HAL_TIM_SET_AUTORELOAD(tim,val);
			__HAL_TIM_SET_COUNTER(tim,0);

			if(HAL_TIM_Base_GetState(tim) == HAL_TIM_STATE_READY) HAL_TIM_Base_Start_IT(tim);
		}
	};
	inline auto get_timer_period = [](TIM_HandleTypeDef *tim)->uint16_t{
		if(HAL_TIM_Base_GetState(tim) == HAL_TIM_STATE_BUSY){
			return __HAL_TIM_GET_AUTORELOAD(tim);
		}else{
			return 0;
		}
	};

	//LEDs
	inline auto LED_R = RmcLib::LEDPWM{&htim5,TIM_CHANNEL_1};
	inline auto LED_G = RmcLib::LEDPWM{&htim5,TIM_CHANNEL_2};
	inline auto LED_B = RmcLib::LEDPWM{&htim5,TIM_CHANNEL_3};


	inline auto LED = std::array<RmcLib::LEDGPIO,MOTOR_N>{
		RmcLib::LEDGPIO{RM_LED1_GPIO_Port,RM_LED1_Pin},
		RmcLib::LEDGPIO{RM_LED2_GPIO_Port,RM_LED2_Pin},
		RmcLib::LEDGPIO{RM_LED3_GPIO_Port,RM_LED3_Pin},
		RmcLib::LEDGPIO{RM_LED4_GPIO_Port,RM_LED4_Pin},
	};

	//can
	inline auto can_main = CommonLib::CanComm<4,4>{&hcan2,CAN_RX_FIFO1,CAN_FILTER_FIFO1,CAN_IT_RX_FIFO1_MSG_PENDING};
	inline auto can_c6x0 = CommonLib::CanComm<4,4>{&hcan1,CAN_RX_FIFO0,CAN_FILTER_FIFO0,CAN_IT_RX_FIFO0_MSG_PENDING};

	//motors
	inline auto driver = std::array<RmcLib::MotorDriver,MOTOR_N>{};
	inline auto motor_state = std::array<RmcLib::C6x0State,MOTOR_N>{
		RmcLib::C6x0State{36.0f},
		RmcLib::C6x0State{36.0f},
		RmcLib::C6x0State{36.0f},
		RmcLib::C6x0State{36.0f}
	};

	//usb
	inline auto usb_cdc = CommonLib::UsbCdcComm<4,4>{&hUsbDeviceFS};

	//monitor
	inline auto monitor = std::array<std::bitset<0x35+1>,MOTOR_N>{};

	inline auto set_p_gain = [](RmcLib::PIDGain g,float p)->RmcLib::PIDGain {g.kp = p; return g;};
	inline auto set_i_gain = [](RmcLib::PIDGain g,float i)->RmcLib::PIDGain {g.kp = i; return g;};
	inline auto set_d_gain = [](RmcLib::PIDGain g,float d)->RmcLib::PIDGain {g.kp = d; return g;};

	inline auto id_map = std::array<CommonLib::IDMap,MOTOR_N>{
		CommonLib::IDMapBuilder()
			.add((uint16_t)RmcReg::CONTROL_TYPE,  CommonLib::DataAccessor::generate<RmcLib::ControlMode>([](RmcLib::ControlMode m){driver[0].set_control_mode(m);},[]()->RmcLib::ControlMode{return driver[0].get_control_mode();}))
			.add((uint16_t)RmcReg::GEAR_RATIO,    CommonLib::DataAccessor::generate<float>([](float ratio){motor_state[0].set_gear_ratio(ratio);},[]()->float{return motor_state[0].get_gear_ratio();}))
			.add((uint16_t)RmcReg::CAN_TIMEOUT,   CommonLib::DataAccessor::generate<float>([](uint16_t period){set_timer_period(can_timeout_timer,period);},[]()->uint16_t{return get_timer_period(can_timeout_timer);}))

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
			.build(),

		CommonLib::IDMapBuilder()
			.add((uint16_t)RmcReg::CONTROL_TYPE,  CommonLib::DataAccessor::generate<RmcLib::ControlMode>([](RmcLib::ControlMode m){driver[1].set_control_mode(m);},[]()->RmcLib::ControlMode{return driver[1].get_control_mode();}))
			.add((uint16_t)RmcReg::GEAR_RATIO,    CommonLib::DataAccessor::generate<float>([](float ratio){motor_state[1].set_gear_ratio(ratio);},[]()->float{return motor_state[1].get_gear_ratio();}))
			.add((uint16_t)RmcReg::CAN_TIMEOUT,   CommonLib::DataAccessor::generate<float>([](uint16_t period){set_timer_period(can_timeout_timer,period);},[]()->uint16_t{return get_timer_period(can_timeout_timer);}))

			.add((uint16_t)RmcReg::PWM,           CommonLib::DataAccessor::generate<float>([]()->float{return driver[1].get_pwm();}))
			.add((uint16_t)RmcReg::PWM_TARGET,    CommonLib::DataAccessor::generate<float>([](float pwm){driver[1].set_pwm(pwm);},[]()->float{return driver[1].get_pwm();}))

			.add((uint16_t)RmcReg::SPD,           CommonLib::DataAccessor::generate<float>([]()->float{return driver[1].get_current_speed();}))
			.add((uint16_t)RmcReg::SPD_TARGET,    CommonLib::DataAccessor::generate<float>([](float st){driver[1].set_target_speed(st);},[]()->float{return driver[1].get_target_speed();}))
			.add((uint16_t)RmcReg::PWM_LIM,       CommonLib::DataAccessor::generate<float>([](float pl){driver[1].set_pwm_limit(pl);}))
			.add((uint16_t)RmcReg::SPD_GAIN_P,    CommonLib::DataAccessor::generate<float>([](float sp){driver[1].set_speed_gain(set_p_gain(driver[1].get_speed_gain(),sp));},[]()->float{return driver[1].get_speed_gain().kp;}))
			.add((uint16_t)RmcReg::SPD_GAIN_I,    CommonLib::DataAccessor::generate<float>([](float si){driver[1].set_speed_gain(set_i_gain(driver[1].get_speed_gain(),si));},[]()->float{return driver[1].get_speed_gain().ki;}))
			.add((uint16_t)RmcReg::SPD_GAIN_D,    CommonLib::DataAccessor::generate<float>([](float sd){driver[1].set_speed_gain(set_d_gain(driver[1].get_speed_gain(),sd));},[]()->float{return driver[1].get_speed_gain().kd;}))

			.add((uint16_t)RmcReg::POS,           CommonLib::DataAccessor::generate<float>([](float pos){driver[1].set_origin(driver[1].get_current_low_position()-pos);},[]()->float{return driver[1].get_current_position();}))
			.add((uint16_t)RmcReg::POS_TARGET,    CommonLib::DataAccessor::generate<float>([](float pt){driver[1].set_target_position(pt);},[]()->float{return driver[1].get_target_position();}))
			.add((uint16_t)RmcReg::SPD_LIM,       CommonLib::DataAccessor::generate<float>([](float sl){driver[1].set_speed_limit(sl);}))
			.add((uint16_t)RmcReg::POS_GAIN_P,    CommonLib::DataAccessor::generate<float>([](float pp){driver[1].set_position_gain(set_p_gain(driver[1].get_position_gain(),pp));},[]()->float{return driver[1].get_position_gain().kp;}))
			.add((uint16_t)RmcReg::POS_GAIN_I,    CommonLib::DataAccessor::generate<float>([](float pi){driver[1].set_position_gain(set_i_gain(driver[1].get_position_gain(),pi));},[]()->float{return driver[1].get_position_gain().ki;}))
			.add((uint16_t)RmcReg::POS_GAIN_D,    CommonLib::DataAccessor::generate<float>([](float pd){driver[1].set_position_gain(set_d_gain(driver[1].get_position_gain(),pd));},[]()->float{return driver[1].get_position_gain().kd;}))

			.add((uint16_t)RmcReg::MONITOR_PERIOD,CommonLib::DataAccessor::generate<uint16_t>([](uint16_t period){set_timer_period(monitor_timer,period);}, []()->uint16_t{return get_timer_period(monitor_timer);}))
			.add((uint16_t)RmcReg::MONITOR_REG,   CommonLib::DataAccessor::generate<uint64_t>([](uint64_t val){ monitor[1] = std::bitset<0x35+1>{val};}, []()->uint64_t{ return monitor[1].to_ullong();}))
			.build(),

		CommonLib::IDMapBuilder()
			.add((uint16_t)RmcReg::CONTROL_TYPE,  CommonLib::DataAccessor::generate<RmcLib::ControlMode>([](RmcLib::ControlMode m){driver[2].set_control_mode(m);},[]()->RmcLib::ControlMode{return driver[2].get_control_mode();}))
			.add((uint16_t)RmcReg::GEAR_RATIO,    CommonLib::DataAccessor::generate<float>([](float ratio){motor_state[2].set_gear_ratio(ratio);},[]()->float{return motor_state[2].get_gear_ratio();}))
			.add((uint16_t)RmcReg::CAN_TIMEOUT,   CommonLib::DataAccessor::generate<float>([](uint16_t period){set_timer_period(can_timeout_timer,period);},[]()->uint16_t{return get_timer_period(can_timeout_timer);}))

			.add((uint16_t)RmcReg::PWM,           CommonLib::DataAccessor::generate<float>([]()->float{return driver[2].get_pwm();}))
			.add((uint16_t)RmcReg::PWM_TARGET,    CommonLib::DataAccessor::generate<float>([](float pwm){driver[2].set_pwm(pwm);},[]()->float{return driver[2].get_pwm();}))

			.add((uint16_t)RmcReg::SPD,           CommonLib::DataAccessor::generate<float>([]()->float{return driver[2].get_current_speed();}))
			.add((uint16_t)RmcReg::SPD_TARGET,    CommonLib::DataAccessor::generate<float>([](float st){driver[2].set_target_speed(st);},[]()->float{return driver[2].get_target_speed();}))
			.add((uint16_t)RmcReg::PWM_LIM,       CommonLib::DataAccessor::generate<float>([](float pl){driver[2].set_pwm_limit(pl);}))
			.add((uint16_t)RmcReg::SPD_GAIN_P,    CommonLib::DataAccessor::generate<float>([](float sp){driver[2].set_speed_gain(set_p_gain(driver[2].get_speed_gain(),sp));},[]()->float{return driver[2].get_speed_gain().kp;}))
			.add((uint16_t)RmcReg::SPD_GAIN_I,    CommonLib::DataAccessor::generate<float>([](float si){driver[2].set_speed_gain(set_i_gain(driver[2].get_speed_gain(),si));},[]()->float{return driver[2].get_speed_gain().ki;}))
			.add((uint16_t)RmcReg::SPD_GAIN_D,    CommonLib::DataAccessor::generate<float>([](float sd){driver[2].set_speed_gain(set_d_gain(driver[2].get_speed_gain(),sd));},[]()->float{return driver[2].get_speed_gain().kd;}))

			.add((uint16_t)RmcReg::POS,           CommonLib::DataAccessor::generate<float>([](float pos){driver[2].set_origin(driver[2].get_current_low_position()-pos);},[]()->float{return driver[2].get_current_position();}))
			.add((uint16_t)RmcReg::POS_TARGET,    CommonLib::DataAccessor::generate<float>([](float pt){driver[2].set_target_position(pt);},[]()->float{return driver[2].get_target_position();}))
			.add((uint16_t)RmcReg::SPD_LIM,       CommonLib::DataAccessor::generate<float>([](float sl){driver[2].set_speed_limit(sl);}))
			.add((uint16_t)RmcReg::POS_GAIN_P,    CommonLib::DataAccessor::generate<float>([](float pp){driver[2].set_position_gain(set_p_gain(driver[2].get_position_gain(),pp));},[]()->float{return driver[2].get_position_gain().kp;}))
			.add((uint16_t)RmcReg::POS_GAIN_I,    CommonLib::DataAccessor::generate<float>([](float pi){driver[2].set_position_gain(set_i_gain(driver[2].get_position_gain(),pi));},[]()->float{return driver[2].get_position_gain().ki;}))
			.add((uint16_t)RmcReg::POS_GAIN_D,    CommonLib::DataAccessor::generate<float>([](float pd){driver[2].set_position_gain(set_d_gain(driver[2].get_position_gain(),pd));},[]()->float{return driver[2].get_position_gain().kd;}))

			.add((uint16_t)RmcReg::MONITOR_PERIOD,CommonLib::DataAccessor::generate<uint16_t>([](uint16_t period){set_timer_period(monitor_timer,period);}, []()->uint16_t{return get_timer_period(monitor_timer);}))
			.add((uint16_t)RmcReg::MONITOR_REG,   CommonLib::DataAccessor::generate<uint64_t>([](uint64_t val){ monitor[2] = std::bitset<0x35+1>{val};}, []()->uint64_t{ return monitor[2].to_ullong();}))
			.build(),

		CommonLib::IDMapBuilder()
			.add((uint16_t)RmcReg::CONTROL_TYPE,  CommonLib::DataAccessor::generate<RmcLib::ControlMode>([](RmcLib::ControlMode m){driver[3].set_control_mode(m);},[]()->RmcLib::ControlMode{return driver[3].get_control_mode();}))
			.add((uint16_t)RmcReg::GEAR_RATIO,    CommonLib::DataAccessor::generate<float>([](float ratio){motor_state[3].set_gear_ratio(ratio);},[]()->float{return motor_state[3].get_gear_ratio();}))
			.add((uint16_t)RmcReg::CAN_TIMEOUT,   CommonLib::DataAccessor::generate<float>([](uint16_t period){set_timer_period(can_timeout_timer,period);},[]()->uint16_t{return get_timer_period(can_timeout_timer);}))

			.add((uint16_t)RmcReg::PWM,           CommonLib::DataAccessor::generate<float>([]()->float{return driver[3].get_pwm();}))
			.add((uint16_t)RmcReg::PWM_TARGET,    CommonLib::DataAccessor::generate<float>([](float pwm){driver[3].set_pwm(pwm);},[]()->float{return driver[3].get_pwm();}))

			.add((uint16_t)RmcReg::SPD,           CommonLib::DataAccessor::generate<float>([]()->float{return driver[3].get_current_speed();}))
			.add((uint16_t)RmcReg::SPD_TARGET,    CommonLib::DataAccessor::generate<float>([](float st){driver[3].set_target_speed(st);},[]()->float{return driver[3].get_target_speed();}))
			.add((uint16_t)RmcReg::PWM_LIM,       CommonLib::DataAccessor::generate<float>([](float pl){driver[3].set_pwm_limit(pl);}))
			.add((uint16_t)RmcReg::SPD_GAIN_P,    CommonLib::DataAccessor::generate<float>([](float sp){driver[3].set_speed_gain(set_p_gain(driver[3].get_speed_gain(),sp));},[]()->float{return driver[3].get_speed_gain().kp;}))
			.add((uint16_t)RmcReg::SPD_GAIN_I,    CommonLib::DataAccessor::generate<float>([](float si){driver[3].set_speed_gain(set_i_gain(driver[3].get_speed_gain(),si));},[]()->float{return driver[3].get_speed_gain().ki;}))
			.add((uint16_t)RmcReg::SPD_GAIN_D,    CommonLib::DataAccessor::generate<float>([](float sd){driver[3].set_speed_gain(set_d_gain(driver[3].get_speed_gain(),sd));},[]()->float{return driver[3].get_speed_gain().kd;}))

			.add((uint16_t)RmcReg::POS,           CommonLib::DataAccessor::generate<float>([](float pos){driver[3].set_origin(driver[3].get_current_low_position()-pos);},[]()->float{return driver[3].get_current_position();}))
			.add((uint16_t)RmcReg::POS_TARGET,    CommonLib::DataAccessor::generate<float>([](float pt){driver[3].set_target_position(pt);},[]()->float{return driver[3].get_target_position();}))
			.add((uint16_t)RmcReg::SPD_LIM,       CommonLib::DataAccessor::generate<float>([](float sl){driver[3].set_speed_limit(sl);}))
			.add((uint16_t)RmcReg::POS_GAIN_P,    CommonLib::DataAccessor::generate<float>([](float pp){driver[3].set_position_gain(set_p_gain(driver[3].get_position_gain(),pp));},[]()->float{return driver[3].get_position_gain().kp;}))
			.add((uint16_t)RmcReg::POS_GAIN_I,    CommonLib::DataAccessor::generate<float>([](float pi){driver[3].set_position_gain(set_i_gain(driver[3].get_position_gain(),pi));},[]()->float{return driver[3].get_position_gain().ki;}))
			.add((uint16_t)RmcReg::POS_GAIN_D,    CommonLib::DataAccessor::generate<float>([](float pd){driver[3].set_position_gain(set_d_gain(driver[3].get_position_gain(),pd));},[]()->float{return driver[3].get_position_gain().kd;}))

			.add((uint16_t)RmcReg::MONITOR_PERIOD,CommonLib::DataAccessor::generate<uint16_t>([](uint16_t period){set_timer_period(monitor_timer,period);}, []()->uint16_t{return get_timer_period(monitor_timer);}))
			.add((uint16_t)RmcReg::MONITOR_REG,   CommonLib::DataAccessor::generate<uint64_t>([](uint64_t val){ monitor[3] = std::bitset<0x35+1>{val};}, []()->uint64_t{ return monitor[3].to_ullong();}))
			.build(),
	};

	//functions
	uint8_t read_board_id(void);

	//各クラス起動処理
	void init(void);

	//受信したモーター情報の処理
	void motor_data_process(void);

	//PWM値の送信
	void send_motor_parameters(void);

	//メインCANの処理（外部との通信）
	void main_comm_prossess(void);

	void execute_common_command(const CommonLib::DataPacket &data);

	void monitor_task(void);

#ifdef MOTOR_DEBUG
	void motor_test(void);
#endif


}

#endif /* BOARD_TASK_HPP_ */

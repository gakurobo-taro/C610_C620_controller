/*
 * board_task.hpp
 *
 *  Created on: 2024/01/02
 *      Author: yaa3k
 */

#ifndef BOARD_TASK_HPP_
#define BOARD_TASK_HPP_


#include "motor_unit.hpp"

#include "LED_control.hpp"
#include "LED_pattern.hpp"
#include "STM32HAL_CommonLib/can_comm.hpp"
#include "STM32HAL_CommonLib/data_packet.hpp"
#include "STM32HAL_CommonLib/data_convert.hpp"
#include "STM32HAL_CommonLib/serial_comm.hpp"


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
	inline auto motor_control_timer = RmcLib::InterruptionTimerHard(&htim14);
	inline auto monitor_timer = RmcLib::InterruptionTimerHard(&htim13);
	inline auto can_timeout_timer = RmcLib::InterruptionTimerHard(&htim12);

	//LEDs
	inline auto LED_R = RmcLib::LEDPWM{&htim5,TIM_CHANNEL_1};
	inline auto LED_G = RmcLib::LEDPWM{&htim5,TIM_CHANNEL_2};
	inline auto LED_B = RmcLib::LEDPWM{&htim5,TIM_CHANNEL_3};

	//can
	inline auto can_main = CommonLib::CanComm<4,4>{&hcan2,CAN_RX_FIFO1,CAN_FILTER_FIFO1,CAN_IT_RX_FIFO1_MSG_PENDING};
	inline auto can_c6x0 = CommonLib::CanComm<4,4>{&hcan1,CAN_RX_FIFO0,CAN_FILTER_FIFO0,CAN_IT_RX_FIFO0_MSG_PENDING};

	//usb
	inline auto usb_cdc = CommonLib::UsbCdcComm<4,4>{&hUsbDeviceFS};

	//driver
	inline auto motor = std::array<RmcBoard::MotorUnit,MOTOR_N>{
		MotorUnitBuilder()
				.set_LED(RM_LED1_GPIO_Port,RM_LED1_Pin)
				.set_gear_ratio(36.0f)
				.set_i2c_encoder(&hi2c3,1000.0f,I2C_SEL1_GPIO_Port,I2C_SEL1_Pin)
				.build(),
		MotorUnitBuilder()
				.set_LED(RM_LED2_GPIO_Port,RM_LED2_Pin)
				.set_gear_ratio(36.0f)
				.set_i2c_encoder(&hi2c3,1000.0f,I2C_SEL2_GPIO_Port,I2C_SEL2_Pin)
				.build(),
		MotorUnitBuilder()
				.set_LED(RM_LED3_GPIO_Port,RM_LED3_Pin)
				.set_gear_ratio(36.0f)
				.set_i2c_encoder(&hi2c3,1000,I2C_SEL3_GPIO_Port,I2C_SEL3_Pin)
				.build(),
		MotorUnitBuilder()
				.set_LED(RM_LED4_GPIO_Port,RM_LED4_Pin)
				.set_gear_ratio(36.0f)
				.set_i2c_encoder(&hi2c3,1000,I2C_SEL4_GPIO_Port,I2C_SEL4_Pin)
				.build(),
	};
	inline auto id_map = std::array<CommonLib::IDMap,MOTOR_N>{
		map_build(motor[0],can_timeout_timer,monitor_timer),
		map_build(motor[1],can_timeout_timer,monitor_timer),
		map_build(motor[2],can_timeout_timer,monitor_timer),
		map_build(motor[3],can_timeout_timer,monitor_timer),
	};

	inline size_t abs_enc_reading_n = 0;

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

	void execute_rmc_command(size_t board_id,const CommonLib::DataPacket &rx_data,CommPort data_from);
	void execute_common_command(size_t board_id,const CommonLib::DataPacket &rx_data,CommPort data_from);

	void monitor_task(void);

	void emergency_stop_sequence(void);
	void emergency_stop_release_sequence(void);

#ifdef MOTOR_DEBUG
	void motor_test(void);
#endif


}

#endif /* BOARD_TASK_HPP_ */

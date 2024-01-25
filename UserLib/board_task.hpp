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
#include "STM32HAL_CommonLib/can_comm.hpp"
#include "STM32HAL_CommonLib/pwm.hpp"
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

	struct GPIOParam{
		GPIO_TypeDef * port;
		uint16_t pin;
		GPIOParam(GPIO_TypeDef * _port,uint16_t _pin):port(_port),pin(_pin){}
	};
	//LEDs
	inline auto LED_R = CommonLib::PWMHard{&htim5,TIM_CHANNEL_1};
	inline auto LED_G = CommonLib::PWMHard{&htim5,TIM_CHANNEL_2};
	inline auto LED_B = CommonLib::PWMHard{&htim5,TIM_CHANNEL_3};

	inline auto LED = std::array<GPIOParam,4>{
		GPIOParam{RM_LED1_GPIO_Port,RM_LED1_Pin},
		GPIOParam{RM_LED2_GPIO_Port,RM_LED2_Pin},
		GPIOParam{RM_LED3_GPIO_Port,RM_LED3_Pin},
		GPIOParam{RM_LED4_GPIO_Port,RM_LED4_Pin},
	};

	inline auto dip_sw = std::array<GPIOParam,4>{
		GPIOParam{ID0_GPIO_Port,ID0_Pin},
		GPIOParam{ID1_GPIO_Port,ID1_Pin},
		GPIOParam{ID2_GPIO_Port,ID2_Pin},
		GPIOParam{ID3_GPIO_Port,ID3_Pin},
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
	inline auto monitor = std::array<std::bitset<0x35>,MOTOR_N>{};
	inline bool monitor_enable = false;

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
	bool write_rmc_command(const CommonLib::DataPacket &data);
	bool read_rmc_command(const CommonLib::DataPacket &data,CommonLib::DataPacket &return_data);
	void execute_common_command(const CommonLib::DataPacket &data);

	void monitor_task(void);

#ifdef MOTOR_DEBUG
	void motor_test(void);
#endif


}

#endif /* BOARD_TASK_HPP_ */

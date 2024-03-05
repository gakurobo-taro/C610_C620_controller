/*
 * board_task.hpp
 *
 *  Created on: 2024/01/02
 *      Author: yaa3k
 */

#ifndef BOARD_TASK_HPP_
#define BOARD_TASK_HPP_

#include "board_info.hpp"


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
	inline auto *motor_control_timer = &htim14;
	inline auto *monitor_timer = &htim13;
	inline auto *can_timeout_timer = &htim12;

	inline bool timeout_en_flag = false;

	inline auto set_timer_period = [](TIM_HandleTypeDef *tim,uint16_t val){
		if(val == 0){
			HAL_TIM_Base_Stop_IT(tim);
		}else{
			__HAL_TIM_SET_AUTORELOAD(tim,val);
			__HAL_TIM_SET_COUNTER(tim,0);

			if(HAL_TIM_Base_GetState(tim) == HAL_TIM_STATE_READY){
				HAL_TIM_Base_Start_IT(tim);
			}
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

	//usb
	inline auto usb_cdc = CommonLib::UsbCdcComm<4,4>{&hUsbDeviceFS};

	//driver


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

	inline auto control_mode_tmp = std::array<RmcLib::ControlMode,MOTOR_N>{};
	void emergency_stop_sequence(void);
	void emergency_stop_release_sequence(void);

#ifdef MOTOR_DEBUG
	void motor_test(void);
#endif


}

#endif /* BOARD_TASK_HPP_ */

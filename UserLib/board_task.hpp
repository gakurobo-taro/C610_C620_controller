/*
 * board_task.hpp
 *
 *  Created on: 2024/01/02
 *      Author: yaa3k
 */

#ifndef BOARD_TASK_HPP_
#define BOARD_TASK_HPP_


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

extern USBD_HandleTypeDef hUsbDeviceFS;

namespace G24_STM32HAL::RmcBoard{

	struct GPIOParam{
		GPIO_TypeDef * port;
		uint16_t pin;
		GPIOParam(GPIO_TypeDef * _port,uint16_t _pin):port(_port),pin(_pin){}
	};
	//LEDs
	CommonLib::PWMHard LED_R = CommonLib::PWMHard{&htim5,TIM_CHANNEL_1};
	CommonLib::PWMHard LED_G = CommonLib::PWMHard{&htim5,TIM_CHANNEL_2};
	CommonLib::PWMHard LED_B = CommonLib::PWMHard{&htim5,TIM_CHANNEL_3};

	std::array<GPIOParam,4> LED{
		GPIOParam(RM_LED1_GPIO_Port,RM_LED1_Pin),
		GPIOParam(RM_LED2_GPIO_Port,RM_LED2_Pin),
		GPIOParam(RM_LED3_GPIO_Port,RM_LED3_Pin),
		GPIOParam(RM_LED4_GPIO_Port,RM_LED4_Pin),
	};

	//can
	CommonLib::CanComm can_main = CommonLib::CanComm(&hcan2,CAN_RX_FIFO1,CAN_FILTER_FIFO1,CAN_IT_RX_FIFO1_MSG_PENDING);
	CommonLib::CanComm can_c6x0 = CommonLib::CanComm(&hcan1,CAN_RX_FIFO0,CAN_FILTER_FIFO0,CAN_IT_RX_FIFO0_MSG_PENDING);

	//motors
	std::array<RmcLib::MotorDriver,4> driver;
	std::array<RmcLib::C6x0State,4> motor_state{
		RmcLib::C6x0State(36.0f),
		RmcLib::C6x0State(36.0f),
		RmcLib::C6x0State(36.0f),
		RmcLib::C6x0State(36.0f)
	};

	//usb
	CommonLib::UsbCdcComm usb_cdc = CommonLib::UsbCdcComm(&hUsbDeviceFS);

	//各クラス起動処理
	void init(void){
		can_main.start();
		can_main.set_filter_free(16);
		can_c6x0.start();
		can_c6x0.set_filter_free(0);
		LED_R.start();
		LED_G.start();
		LED_B.start();

		for(int i = 0; i<4; i++){
			driver.at(i).set_speed_gain(0.2f, 0.002, 0);
			driver.at(i).set_position_gain(1.0f, 0.001, 0);
			driver.at(i).set_speed_limit(-1.0,1.0);
		}
	}

	//受信したモーター情報の処理
	void motor_data_process(void){
		if(can_c6x0.rx_available()){
			CommonLib::CanFrame rx_frame;
			can_c6x0.rx(rx_frame);

			if(rx_frame.id & 0x200){
				int id = (rx_frame.id&0xF)-1;
				motor_state.at(id).update(rx_frame);
				driver.at(id).update_operation_val(motor_state.at(id));
				HAL_GPIO_WritePin(LED.at(id).port,LED.at(id).pin,GPIO_PIN_SET);
			}
		}
	}

	//PWM値の送信
	void send_motor_parameters(void){
		CommonLib::CanFrame tx_frame;
		tx_frame.id = 0x200;
		auto writer = tx_frame.writer();

		for(int i = 0; i < 4; i++){
			int16_t duty = (int16_t)(driver.at(i).get_pwm() * 10000.0f);
			writer.write<uint8_t>(duty>>8);
			writer.write<uint8_t>(duty&0xFF);
		}

		can_c6x0.tx(tx_frame);
		for(size_t i = 0; i < LED.size(); i++){
			HAL_GPIO_WritePin(LED.at(i).port,LED.at(i).pin,GPIO_PIN_RESET);
		}
	}

	//メインCANの処理（外部との通信）
	void main_comm_prossess(void){
		  for(int i = 0; i < 4; i++){
			  driver.at(i).set_control_mode(RmcLib::ControlMode::POSITION_MODE);
			  driver.at(i).set_target_position(-0.3f);
			  driver.at(i).set_speed_limit(-0.2,0.2);
		  }
		  HAL_Delay(1000);

		  for(int i = 0; i < 4; i++){
			  driver.at(i).set_control_mode(RmcLib::ControlMode::POSITION_MODE);
			  driver.at(i).set_target_position(0.3f);
			  driver.at(i).set_speed_limit(-0.1,0.1);
		  }
		  HAL_Delay(2000);

	}



}

#endif /* BOARD_TASK_HPP_ */

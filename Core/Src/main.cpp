/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <array>

#include "../../UserLib/motor_control.hpp"

#include "../../UserLib/STM32HAL_CommonLib/can_comm.hpp"
#include "../../UserLib/STM32HAL_CommonLib/pwm.hpp"
#include "../../UserLib/STM32HAL_CommonLib/data_packet.hpp"
#include "../../UserLib/STM32HAL_CommonLib/data_convert.hpp"
#include "../../UserLib/STM32HAL_CommonLib/serial_comm.hpp"

using namespace G24_STM32HAL::CommonLib;
using namespace G24_STM32HAL::RmcLib;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PWMHard LED_R(&htim5,TIM_CHANNEL_1);
PWMHard LED_G(&htim5,TIM_CHANNEL_2);
PWMHard LED_B(&htim5,TIM_CHANNEL_3);

CanComm can_main = CanComm(&hcan2,CAN_RX_FIFO1,CAN_FILTER_FIFO1,CAN_IT_RX_FIFO1_MSG_PENDING);
CanComm can_c6x0 = CanComm(&hcan1,CAN_RX_FIFO0,CAN_FILTER_FIFO0,CAN_IT_RX_FIFO0_MSG_PENDING);

std::array<MotorDriver,4> motor;

std::array<C6x0State,4> motor_state{
		C6x0State(36.0f),
		C6x0State(36.0f),
		C6x0State(36.0f),
		C6x0State(36.0f)
};

std::array<GPIO_TypeDef *,4> LED_port {RM_LED1_GPIO_Port,RM_LED2_GPIO_Port,RM_LED3_GPIO_Port,RM_LED4_GPIO_Port};
std::array<uint16_t,4> LED_pin {RM_LED1_Pin,RM_LED2_Pin,RM_LED3_Pin,RM_LED4_Pin};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	can_c6x0.rx_interrupt_task();

	CanFrame rx_frame;
	can_c6x0.rx(rx_frame);

	if(rx_frame.id & 0x200){
		int id = (rx_frame.id&0xF)-1;
		motor_state.at(id).update(rx_frame);
		motor.at(id).update_operation_val(motor_state.at(id));
		HAL_GPIO_WritePin(LED_port[id],LED_pin[id],GPIO_PIN_SET);
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	can_main.rx_interrupt_task();
}



extern USBD_HandleTypeDef hUsbDeviceFS;
UsbCdcComm usb_cdc(&hUsbDeviceFS);

void usb_cdc_rx_callback(const uint8_t *input,size_t size){
	usb_cdc.rx_interrupt_task(input, size);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim14){
    	usb_cdc.tx_interrupt_task();

    	CanFrame tx_frame;
    	tx_frame.id = 0x200;
    	auto writer = tx_frame.writer();

    	for(int i = 0; i < 4; i++){
    		int16_t duty = (int16_t)(motor.at(i).get_pwm() * 10000.0f);
    		writer.write<uint8_t>(duty>>8);
    		writer.write<uint8_t>(duty&0xFF);
    	}

    	can_c6x0.tx(tx_frame);

    	LED_R.out_as_gpio_toggle();
    	HAL_GPIO_WritePin(RM_LED1_GPIO_Port,RM_LED1_Pin,GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(RM_LED2_GPIO_Port,RM_LED2_Pin,GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(RM_LED3_GPIO_Port,RM_LED3_Pin,GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(RM_LED4_GPIO_Port,RM_LED4_Pin,GPIO_PIN_RESET);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C3_Init();
  MX_TIM5_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  can_main.start();
  can_main.set_filter_free(16);
  can_c6x0.start();
  can_c6x0.set_filter_free(0);
  LED_R.start();
  LED_G.start();
  LED_B.start();

  HAL_TIM_Base_Start_IT(&htim14);

  char str[64] = {0};

  for(int i = 0; i<4; i++){
	  motor.at(i).set_speed_gain(0.2f, 0.002, 0);
	  motor.at(i).set_position_gain(1.0f, 0.001, 0);
	  motor.at(i).set_speed_limit(-1.0,1.0);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  for(int i = 0; i < 4; i++){
		  motor.at(i).set_control_mode(ControlMode::POSITION_MODE);
		  motor.at(i).set_target_position(-0.3f);
		  motor.at(i).set_speed_limit(-0.2,0.2);

//		  motor[i].set_control_mode(ControlMode::SPEED_MODE);
//		  motor[i].set_target_speed(0.5f);

//		  motor[i].set_control_mode(ControlMode::PWM_MODE);
//		  motor[i].set_pwm(0.05f);
	  }
	  HAL_Delay(1000);

	  for(int i = 0; i < 4; i++){
		  motor.at(i).set_control_mode(ControlMode::POSITION_MODE);
		  motor.at(i).set_target_position(0.3f);
		  motor.at(i).set_speed_limit(-0.1,0.1);

//		  motor[i].set_control_mode(ControlMode::SPEED_MODE);
//		  motor[i].set_target_speed(-0.2f);

//		  motor[i].set_control_mode(ControlMode::PWM_MODE);
//		  motor[i].set_pwm(-0.05f);
	  }
	  HAL_Delay(2000);

//
//	  motor[1].set_control_mode(ControlMode::SPEED_MODE);
//	  motor[1].set_target_speed(0.5f);

//	  motor[0].set_control_mode(ControlMode::PWM_MODE);
//	  motor[0].set_pwm(0.05f);

//	  sprintf(str,"mode:%d,pwm:%4.3f,speed:%4.3f,target_speed:%4.3f\r\n",
//			  (int)motor.get_control_mode(),motor.get_pwm(),motor.get_current_speed(),motor.get_target_speed());
//	  usb_cdc.tx((uint8_t*)str,strlen(str));

//	  HAL_Delay(1);
//	  sprintf(str,"angle:%d,speed:%d\r\n",motor_state.angle,motor_state.speed);
//	  usb_cdc.tx((uint8_t*)str,strlen(str));

//	  sprintf(str,"%4.3f,%4.3f,%4.3f,%4.3f\r\n",
//			  motor[0].get_pwm(),motor[0].get_current_speed(),motor[0].get_target_speed(),motor[0].get_current_position());
	  sprintf(str,"%4.3f,%4.3f,%4.3f,%4.3f\r\n",motor_state.at(0).rad,motor_state.at(0).speed,motor_state.at(1).rad,motor_state.at(1).speed);

	  usb_cdc.tx((uint8_t*)str,strlen(str));
	  LED_G.out_as_gpio_toggle();
	  HAL_Delay(2);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

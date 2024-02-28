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
#include "../../UserLib/board_task.hpp"

using namespace G24_STM32HAL;
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

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	RmcBoard::LED_B.out_as_gpio(true);
	if(hcan == RmcBoard::can_main.get_can_handle()){
		RmcBoard::can_main.tx_interrupt_task();
	}else if(hcan == RmcBoard::can_c6x0.get_can_handle()){
		RmcBoard::can_c6x0.tx_interrupt_task();
	}
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	RmcBoard::LED_B.out_as_gpio(true);
	if(hcan == RmcBoard::can_main.get_can_handle()){
		RmcBoard::can_main.tx_interrupt_task();
	}else if(hcan == RmcBoard::can_c6x0.get_can_handle()){
		RmcBoard::can_c6x0.tx_interrupt_task();
	}
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	RmcBoard::LED_B.out_as_gpio(true);
	if(hcan == RmcBoard::can_main.get_can_handle()){
		RmcBoard::can_main.tx_interrupt_task();
	}else if(hcan == RmcBoard::can_c6x0.get_can_handle()){
		RmcBoard::can_c6x0.tx_interrupt_task();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	RmcBoard::LED_B.out_as_gpio(true);
	RmcBoard::can_c6x0.rx_interrupt_task();
	RmcBoard::motor_data_process();
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	RmcBoard::LED_B.out_as_gpio(true);
	RmcBoard::can_main.rx_interrupt_task();
	__HAL_TIM_SET_COUNTER(&htim12,0);
}

void usb_cdc_rx_callback(const uint8_t *input,size_t size){
	RmcBoard::LED_B.out_as_gpio(true);
	RmcBoard::usb_cdc.rx_interrupt_task(input, size);
	__HAL_TIM_SET_COUNTER(&htim12,0);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == RmcBoard::motor_control_timer){
    	RmcBoard::LED_G.out_as_gpio(true);
    	RmcBoard::usb_cdc.tx_interrupt_task();
    	RmcBoard::send_motor_parameters();

    }else if(htim == RmcBoard::monitor_timer){
    	RmcBoard::LED_R.out_as_gpio(true);
    	RmcBoard::monitor_task();

    }else if(htim == RmcBoard::can_timeout_timer){
    	RmcBoard::LED_R.out_as_gpio(true);
    	for(int i = 0; i<5000;i++){

    	}
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
  MX_TIM13_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  RmcBoard::init();

  HAL_TIM_Base_Start_IT(RmcBoard::motor_control_timer);
  //HAL_TIM_Base_Start_IT(RmcBoard::monitor_timer);
  HAL_TIM_Base_Start_IT(RmcBoard::can_timeout_timer);
  __HAL_TIM_SET_AUTORELOAD(RmcBoard::monitor_timer, 700);
  __HAL_TIM_SET_AUTORELOAD(RmcBoard::can_timeout_timer, 2000);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  RmcBoard::main_comm_prossess();

	  RmcBoard::LED_R.out_as_gpio(false);
	  RmcBoard::LED_G.out_as_gpio(false);
	  RmcBoard::LED_B.out_as_gpio(false);

//	  G24_STM32HAL::CommonLib::DataPacket data;
//	  data.board_ID = read_board_id();
//	  data.is_request = false;
//	  data.data_type = G24_STM32HAL::CommonLib::DataType::RMC_DATA;
//	  data.register_ID = 0x11;
//	  auto writer = data.writer();
//	  writer.write<float>(0.01f);
//	  execute_rmc_command(data);
//	  HAL_Delay(100);

	  //motor_test();
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

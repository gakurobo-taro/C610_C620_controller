/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define USE_USB_CDC
void usb_cdc_rx_callback(const uint8_t *input,size_t size);
#define USE_CAN
void SystemClock_Config(void);

#define MOTOR_DEBUG

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_0
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_2
#define LED_B_GPIO_Port GPIOA
#define RM_LED4_Pin GPIO_PIN_4
#define RM_LED4_GPIO_Port GPIOA
#define RM_LED3_Pin GPIO_PIN_5
#define RM_LED3_GPIO_Port GPIOA
#define RM_LED2_Pin GPIO_PIN_6
#define RM_LED2_GPIO_Port GPIOA
#define RM_LED1_Pin GPIO_PIN_7
#define RM_LED1_GPIO_Port GPIOA
#define ID0_Pin GPIO_PIN_4
#define ID0_GPIO_Port GPIOC
#define ID2_Pin GPIO_PIN_5
#define ID2_GPIO_Port GPIOC
#define ID1_Pin GPIO_PIN_0
#define ID1_GPIO_Port GPIOB
#define ID3_Pin GPIO_PIN_1
#define ID3_GPIO_Port GPIOB
#define I2C_SEL4_Pin GPIO_PIN_15
#define I2C_SEL4_GPIO_Port GPIOB
#define I2C_SEL3_Pin GPIO_PIN_6
#define I2C_SEL3_GPIO_Port GPIOC
#define I2C_SEL2_Pin GPIO_PIN_7
#define I2C_SEL2_GPIO_Port GPIOC
#define I2C_SEL1_Pin GPIO_PIN_8
#define I2C_SEL1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
extern int usb_rx_flag;
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NAND0_PAGE_SIZE 0x800
#define NAND0_SPARE_AREA_SIZE 0x40
#define NAND0_BLOCK_SIZE 0x40
#define NAND0_BLOCK_NUMBER 0x400
#define NAND0_PLAN_SIZE 0x400
#define NAND0_PLAN_NUMBER 1
#define TEST_1_Pin GPIO_PIN_12
#define TEST_1_GPIO_Port GPIOH
#define TEST_2_Pin GPIO_PIN_15
#define TEST_2_GPIO_Port GPIOB
#define TX_DEBUG_Pin GPIO_PIN_8
#define TX_DEBUG_GPIO_Port GPIOD
#define RX_DEBUG_Pin GPIO_PIN_9
#define RX_DEBUG_GPIO_Port GPIOD
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define TX_EN_DYNAMIXEL_Pin GPIO_PIN_0
#define TX_EN_DYNAMIXEL_GPIO_Port GPIOI
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define RE_DE_485_Pin GPIO_PIN_15
#define RE_DE_485_GPIO_Port GPIOA
#define TX_DYNAMIXEL_Pin GPIO_PIN_10
#define TX_DYNAMIXEL_GPIO_Port GPIOC
#define RX_DYNAMIXEL_Pin GPIO_PIN_11
#define RX_DYNAMIXEL_GPIO_Port GPIOC
#define TX_485_Pin GPIO_PIN_12
#define TX_485_GPIO_Port GPIOC
#define RX_485_Pin GPIO_PIN_2
#define RX_485_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

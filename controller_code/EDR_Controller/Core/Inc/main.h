/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32h7xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FLASH_IO2_Pin GPIO_PIN_2
#define FLASH_IO2_GPIO_Port GPIOE
#define LED_PIN_Pin GPIO_PIN_4
#define LED_PIN_GPIO_Port GPIOC
#define MCU_HSE_IN_Pin GPIO_PIN_0
#define MCU_HSE_IN_GPIO_Port GPIOH
#define MCU_HSE_OUT_Pin GPIO_PIN_1
#define MCU_HSE_OUT_GPIO_Port GPIOH
#define DISP_TX_Pin GPIO_PIN_0
#define DISP_TX_GPIO_Port GPIOA
#define DISP_RX_Pin GPIO_PIN_1
#define DISP_RX_GPIO_Port GPIOA
#define MCU_Pneu_3_Pin GPIO_PIN_4
#define MCU_Pneu_3_GPIO_Port GPIOA
#define MCU_Pneu_5_1_Pin GPIO_PIN_5
#define MCU_Pneu_5_1_GPIO_Port GPIOA
#define MCU_Pneu_5_2_Pin GPIO_PIN_6
#define MCU_Pneu_5_2_GPIO_Port GPIOA
#define BUCK_PG_Pin GPIO_PIN_7
#define BUCK_PG_GPIO_Port GPIOA
#define FLASH_CLK_Pin GPIO_PIN_2
#define FLASH_CLK_GPIO_Port GPIOB
#define FLASH_NCS_Pin GPIO_PIN_10
#define FLASH_NCS_GPIO_Port GPIOB
#define FLASH_MOSI_IO0_Pin GPIO_PIN_11
#define FLASH_MOSI_IO0_GPIO_Port GPIOD
#define FLASH_MISO_IO1_Pin GPIO_PIN_12
#define FLASH_MISO_IO1_GPIO_Port GPIOD
#define FLASH_IO3_Pin GPIO_PIN_13
#define FLASH_IO3_GPIO_Port GPIOD
#define MCU_D_N_Pin GPIO_PIN_11
#define MCU_D_N_GPIO_Port GPIOA
#define MCU_D_P_Pin GPIO_PIN_12
#define MCU_D_P_GPIO_Port GPIOA
#define MCU__SWDIO_Pin GPIO_PIN_13
#define MCU__SWDIO_GPIO_Port GPIOA
#define MCU_CLK_Pin GPIO_PIN_14
#define MCU_CLK_GPIO_Port GPIOA
#define MCU_ODRIVE_TX_Pin GPIO_PIN_12
#define MCU_ODRIVE_TX_GPIO_Port GPIOC
#define MCU_ODRIVE_RX_Pin GPIO_PIN_5
#define MCU_ODRIVE_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

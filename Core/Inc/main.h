/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  *                   Main program for SODAQ LOGGER
  * @author         : Phuoc K. Ly
  * @version        : 0.1
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define FRAM_HOLD_Pin GPIO_PIN_4
#define FRAM_HOLD_GPIO_Port GPIOA
#define FRAM_CS_Pin GPIO_PIN_5
#define FRAM_CS_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_13
#define SD_CS_GPIO_Port GPIOB
#define SD_EN_Pin GPIO_PIN_8
#define SD_EN_GPIO_Port GPIOA
#define Vbus_Sense_Pin GPIO_PIN_9
#define Vbus_Sense_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define SPI_FRAM_HANDLE hspi1
#define SD_SPI_HANDLE hspi2
#define USART_HANDLE huart2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

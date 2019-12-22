/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOF
#define ENCODER_PHA_Pin GPIO_PIN_0
#define ENCODER_PHA_GPIO_Port GPIOA
#define ENCODER_PHB_Pin GPIO_PIN_1
#define ENCODER_PHB_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define V16_SENSE_Pin GPIO_PIN_3
#define V16_SENSE_GPIO_Port GPIOA
#define W5500_nSS_Pin GPIO_PIN_4
#define W5500_nSS_GPIO_Port GPIOA
#define MOTOR_BKIN_Pin GPIO_PIN_6
#define MOTOR_BKIN_GPIO_Port GPIOA
#define W5500_nINT_Pin GPIO_PIN_7
#define W5500_nINT_GPIO_Port GPIOA
#define W5500_nRST_Pin GPIO_PIN_0
#define W5500_nRST_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOB
#define MOTOR_A_L_Pin GPIO_PIN_8
#define MOTOR_A_L_GPIO_Port GPIOA
#define MOTOR_B_L_Pin GPIO_PIN_9
#define MOTOR_B_L_GPIO_Port GPIOA
#define MOTOR_A_H_Pin GPIO_PIN_10
#define MOTOR_A_H_GPIO_Port GPIOA
#define MOTOR_B_H_Pin GPIO_PIN_11
#define MOTOR_B_H_GPIO_Port GPIOA
#define LIDAR_MTRCTL_Pin GPIO_PIN_12
#define LIDAR_MTRCTL_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define W5500_SCK_Pin GPIO_PIN_3
#define W5500_SCK_GPIO_Port GPIOB
#define W5500_MISO_Pin GPIO_PIN_4
#define W5500_MISO_GPIO_Port GPIOB
#define W5500_MOSI_Pin GPIO_PIN_5
#define W5500_MOSI_GPIO_Port GPIOB
#define LIDAR_TX_Pin GPIO_PIN_6
#define LIDAR_TX_GPIO_Port GPIOB
#define LIDAR_RX_Pin GPIO_PIN_7
#define LIDAR_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

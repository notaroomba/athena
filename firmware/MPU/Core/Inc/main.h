/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BAR_SCK_Pin GPIO_PIN_2
#define BAR_SCK_GPIO_Port GPIOE
#define BAR_MISO_Pin GPIO_PIN_5
#define BAR_MISO_GPIO_Port GPIOE
#define BAR_MOSI_Pin GPIO_PIN_6
#define BAR_MOSI_GPIO_Port GPIOE
#define TPU_TX_Pin GPIO_PIN_0
#define TPU_TX_GPIO_Port GPIOA
#define TPU_RX_Pin GPIO_PIN_1
#define TPU_RX_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define MCU_MOSI_Pin GPIO_PIN_7
#define MCU_MOSI_GPIO_Port GPIOA
#define TPU_SELECT_Pin GPIO_PIN_4
#define TPU_SELECT_GPIO_Port GPIOC
#define MAG_CS_Pin GPIO_PIN_1
#define MAG_CS_GPIO_Port GPIOB
#define MAG_MOSI_Pin GPIO_PIN_2
#define MAG_MOSI_GPIO_Port GPIOB
#define IMU_FSYNC_Pin GPIO_PIN_9
#define IMU_FSYNC_GPIO_Port GPIOE
#define IMU1_INT_Pin GPIO_PIN_10
#define IMU1_INT_GPIO_Port GPIOE
#define IMU1_CS_Pin GPIO_PIN_11
#define IMU1_CS_GPIO_Port GPIOE
#define IMU2_INT_Pin GPIO_PIN_12
#define IMU2_INT_GPIO_Port GPIOE
#define IMU2_CS_Pin GPIO_PIN_13
#define IMU2_CS_GPIO_Port GPIOE
#define IMU3_INT_Pin GPIO_PIN_14
#define IMU3_INT_GPIO_Port GPIOE
#define IMU3_CS_Pin GPIO_PIN_15
#define IMU3_CS_GPIO_Port GPIOE
#define MPU_R_Pin GPIO_PIN_12
#define MPU_R_GPIO_Port GPIOB
#define MPU_G_Pin GPIO_PIN_13
#define MPU_G_GPIO_Port GPIOB
#define MPU_B_Pin GPIO_PIN_14
#define MPU_B_GPIO_Port GPIOB
#define MPU_TX_EX_Pin GPIO_PIN_9
#define MPU_TX_EX_GPIO_Port GPIOA
#define MPU_RX_EX_Pin GPIO_PIN_10
#define MPU_RX_EX_GPIO_Port GPIOA
#define MAG_SCK_Pin GPIO_PIN_10
#define MAG_SCK_GPIO_Port GPIOC
#define MAG_MISO_Pin GPIO_PIN_11
#define MAG_MISO_GPIO_Port GPIOC
#define MPU_CAN_RX_Pin GPIO_PIN_0
#define MPU_CAN_RX_GPIO_Port GPIOD
#define MPU_CAN_TX_Pin GPIO_PIN_1
#define MPU_CAN_TX_GPIO_Port GPIOD
#define MPU_CAN_S_Pin GPIO_PIN_2
#define MPU_CAN_S_GPIO_Port GPIOD
#define BMP_CS_Pin GPIO_PIN_3
#define BMP_CS_GPIO_Port GPIOD
#define BMP_INT_Pin GPIO_PIN_4
#define BMP_INT_GPIO_Port GPIOD
#define BMP_INT_EXTI_IRQn EXTI4_IRQn
#define ICP_CS_Pin GPIO_PIN_5
#define ICP_CS_GPIO_Port GPIOD
#define ICP_INT_Pin GPIO_PIN_6
#define ICP_INT_GPIO_Port GPIOD
#define IMU_MOSI_Pin GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOD
#define MCU_SCK_Pin GPIO_PIN_3
#define MCU_SCK_GPIO_Port GPIOB
#define MCU_MISO_Pin GPIO_PIN_4
#define MCU_MISO_GPIO_Port GPIOB
#define SPU_SELECT_Pin GPIO_PIN_9
#define SPU_SELECT_GPIO_Port GPIOB
#define SPU_RX_Pin GPIO_PIN_0
#define SPU_RX_GPIO_Port GPIOE
#define SPU_TX_Pin GPIO_PIN_1
#define SPU_TX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

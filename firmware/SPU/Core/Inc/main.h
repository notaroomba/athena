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
#include "stm32g4xx_hal.h"

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
#define SERVO1_PWM_Pin GPIO_PIN_0
#define SERVO1_PWM_GPIO_Port GPIOC
#define SERVO2_PWM_Pin GPIO_PIN_1
#define SERVO2_PWM_GPIO_Port GPIOC
#define SERVO3_PWM_Pin GPIO_PIN_2
#define SERVO3_PWM_GPIO_Port GPIOC
#define SERVO4_PWM_Pin GPIO_PIN_3
#define SERVO4_PWM_GPIO_Port GPIOC
#define SERVO5_PWM_Pin GPIO_PIN_0
#define SERVO5_PWM_GPIO_Port GPIOA
#define SERVO6_PWM_Pin GPIO_PIN_1
#define SERVO6_PWM_GPIO_Port GPIOA
#define PD_IRQ_Pin GPIO_PIN_3
#define PD_IRQ_GPIO_Port GPIOA
#define SPU_SELECT_Pin GPIO_PIN_4
#define SPU_SELECT_GPIO_Port GPIOA
#define MCU_SCK_Pin GPIO_PIN_5
#define MCU_SCK_GPIO_Port GPIOA
#define MCU_MISO_Pin GPIO_PIN_6
#define MCU_MISO_GPIO_Port GPIOA
#define MCU_MOSI_Pin GPIO_PIN_7
#define MCU_MOSI_GPIO_Port GPIOA
#define SPU_TX_EX_Pin GPIO_PIN_4
#define SPU_TX_EX_GPIO_Port GPIOC
#define SPU_RX_EX_Pin GPIO_PIN_5
#define SPU_RX_EX_GPIO_Port GPIOC
#define SERVO1_EN_Pin GPIO_PIN_0
#define SERVO1_EN_GPIO_Port GPIOB
#define SERVO2_EN_Pin GPIO_PIN_1
#define SERVO2_EN_GPIO_Port GPIOB
#define SERVO3_EN_Pin GPIO_PIN_2
#define SERVO3_EN_GPIO_Port GPIOB
#define SPU_B_Pin GPIO_PIN_11
#define SPU_B_GPIO_Port GPIOB
#define SPU_G_Pin GPIO_PIN_12
#define SPU_G_GPIO_Port GPIOB
#define SPU_R_Pin GPIO_PIN_13
#define SPU_R_GPIO_Port GPIOB
#define SERVO4_EN_Pin GPIO_PIN_14
#define SERVO4_EN_GPIO_Port GPIOB
#define SERVO5_EN_Pin GPIO_PIN_15
#define SERVO5_EN_GPIO_Port GPIOB
#define SERVO6_EN_Pin GPIO_PIN_6
#define SERVO6_EN_GPIO_Port GPIOC
#define PYRO_6_Pin GPIO_PIN_7
#define PYRO_6_GPIO_Port GPIOC
#define PYRO_5_Pin GPIO_PIN_8
#define PYRO_5_GPIO_Port GPIOC
#define PYRO_4_Pin GPIO_PIN_9
#define PYRO_4_GPIO_Port GPIOC
#define PYRO_3_Pin GPIO_PIN_8
#define PYRO_3_GPIO_Port GPIOA
#define PYRO_2_Pin GPIO_PIN_9
#define PYRO_2_GPIO_Port GPIOA
#define PYRO_1_Pin GPIO_PIN_10
#define PYRO_1_GPIO_Port GPIOA
#define SPU_PD_SCL_Pin GPIO_PIN_13
#define SPU_PD_SCL_GPIO_Port GPIOA
#define SPU_PD_SDA_Pin GPIO_PIN_14
#define SPU_PD_SDA_GPIO_Port GPIOA
#define SPU_PD_IRQ_Pin GPIO_PIN_15
#define SPU_PD_IRQ_GPIO_Port GPIOA
#define SPU_PROCHOT_Pin GPIO_PIN_10
#define SPU_PROCHOT_GPIO_Port GPIOC
#define CMPOUT_Pin GPIO_PIN_11
#define CMPOUT_GPIO_Port GPIOC
#define SPU_RX_Pin GPIO_PIN_12
#define SPU_RX_GPIO_Port GPIOC
#define SPU_TX_Pin GPIO_PIN_2
#define SPU_TX_GPIO_Port GPIOD
#define EN_OTG_Pin GPIO_PIN_3
#define EN_OTG_GPIO_Port GPIOB
#define CHRG_OK_Pin GPIO_PIN_4
#define CHRG_OK_GPIO_Port GPIOB
#define SPU_CAN_RX_Pin GPIO_PIN_5
#define SPU_CAN_RX_GPIO_Port GPIOB
#define SPU_CAN_TX_Pin GPIO_PIN_6
#define SPU_CAN_TX_GPIO_Port GPIOB
#define SPU_CAN_S_Pin GPIO_PIN_7
#define SPU_CAN_S_GPIO_Port GPIOB
#define RESET_MPU_Pin GPIO_PIN_9
#define RESET_MPU_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Software entry into the ST ROM bootloader (USB DFU) without the BOOT/RESET buttons:
 * the USB console command 'B' stores DFU_MAGIC at DFU_MAGIC_ADDR and resets; the first thing
 * main() does is check it and jump to system memory. 4 KB below the top of SRAM: untouched by the startup code and by the shallow stack right after reset. */
#define DFU_MAGIC       0x41544844UL   /* "ATHD" */
#define DFU_MAGIC_ADDR  0x2001F000UL
#define DFU_SYSMEM_ADDR 0x1FFF0000UL   /* ST system memory (AN2606) */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

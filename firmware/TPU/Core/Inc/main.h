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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FLASH_IO2_Pin GPIO_PIN_2
#define FLASH_IO2_GPIO_Port GPIOE
#define MCU_MISO_Pin GPIO_PIN_5
#define MCU_MISO_GPIO_Port GPIOE
#define MCU_MOSI_Pin GPIO_PIN_6
#define MCU_MOSI_GPIO_Port GPIOE
#define RF_RESET_Pin GPIO_PIN_0
#define RF_RESET_GPIO_Port GPIOA
#define FLASH_IO3_Pin GPIO_PIN_1
#define FLASH_IO3_GPIO_Port GPIOA
#define RF_DIO4_Pin GPIO_PIN_2
#define RF_DIO4_GPIO_Port GPIOA
#define RF_DIO5_Pin GPIO_PIN_3
#define RF_DIO5_GPIO_Port GPIOA
#define RF_CS_Pin GPIO_PIN_4
#define RF_CS_GPIO_Port GPIOA
#define RF_SCK_Pin GPIO_PIN_5
#define RF_SCK_GPIO_Port GPIOA
#define RF_MISO_Pin GPIO_PIN_6
#define RF_MISO_GPIO_Port GPIOA
#define RF_MOSI_Pin GPIO_PIN_7
#define RF_MOSI_GPIO_Port GPIOA
#define RF_DIO3_Pin GPIO_PIN_4
#define RF_DIO3_GPIO_Port GPIOC
#define RF_DIO2_Pin GPIO_PIN_5
#define RF_DIO2_GPIO_Port GPIOC
#define RF_DIO1_Pin GPIO_PIN_0
#define RF_DIO1_GPIO_Port GPIOB
#define RF_DIO0_Pin GPIO_PIN_1
#define RF_DIO0_GPIO_Port GPIOB
#define FLASH_SCK_Pin GPIO_PIN_2
#define FLASH_SCK_GPIO_Port GPIOB
#define BT_RX_Pin GPIO_PIN_7
#define BT_RX_GPIO_Port GPIOE
#define BT_TX_Pin GPIO_PIN_8
#define BT_TX_GPIO_Port GPIOE
#define BT_RESET_Pin GPIO_PIN_9
#define BT_RESET_GPIO_Port GPIOE
#define MCU_SCK_Pin GPIO_PIN_12
#define MCU_SCK_GPIO_Port GPIOE
#define TPU_SELECT_Pin GPIO_PIN_13
#define TPU_SELECT_GPIO_Port GPIOE
#define FLASH_CS_Pin GPIO_PIN_10
#define FLASH_CS_GPIO_Port GPIOB
#define TPU_CAN_RX_Pin GPIO_PIN_12
#define TPU_CAN_RX_GPIO_Port GPIOB
#define TPU_CAN_TX_Pin GPIO_PIN_13
#define TPU_CAN_TX_GPIO_Port GPIOB
#define TPU_CAN_S_Pin GPIO_PIN_14
#define TPU_CAN_S_GPIO_Port GPIOB
#define TPU_R_Pin GPIO_PIN_15
#define TPU_R_GPIO_Port GPIOB
#define TPU_G_Pin GPIO_PIN_8
#define TPU_G_GPIO_Port GPIOD
#define TPU_B_Pin GPIO_PIN_9
#define TPU_B_GPIO_Port GPIOD
#define FLASH_IO0_Pin GPIO_PIN_11
#define FLASH_IO0_GPIO_Port GPIOD
#define FLASH_IO1_Pin GPIO_PIN_12
#define FLASH_IO1_GPIO_Port GPIOD
#define SD_DAT0_Pin GPIO_PIN_8
#define SD_DAT0_GPIO_Port GPIOC
#define SD_DAT1_Pin GPIO_PIN_9
#define SD_DAT1_GPIO_Port GPIOC
#define TPU_TX_EX_Pin GPIO_PIN_9
#define TPU_TX_EX_GPIO_Port GPIOA
#define TPU_RX_EX_Pin GPIO_PIN_10
#define TPU_RX_EX_GPIO_Port GPIOA
#define GPS_TIMEPULSE_Pin GPIO_PIN_15
#define GPS_TIMEPULSE_GPIO_Port GPIOA
#define SD_DAT2_Pin GPIO_PIN_10
#define SD_DAT2_GPIO_Port GPIOC
#define SD_DAT3_Pin GPIO_PIN_11
#define SD_DAT3_GPIO_Port GPIOC
#define SD_SCK_Pin GPIO_PIN_12
#define SD_SCK_GPIO_Port GPIOC
#define SD_CD_Pin GPIO_PIN_1
#define SD_CD_GPIO_Port GPIOD
#define SD_CMD_Pin GPIO_PIN_2
#define SD_CMD_GPIO_Port GPIOD
#define GPS_SEL_Pin GPIO_PIN_3
#define GPS_SEL_GPIO_Port GPIOD
#define GPS_SAFEBOOT_Pin GPIO_PIN_4
#define GPS_SAFEBOOT_GPIO_Port GPIOD
#define GPS_LNA_EN_Pin GPIO_PIN_5
#define GPS_LNA_EN_GPIO_Port GPIOD
#define GPS_MOSI_Pin GPIO_PIN_6
#define GPS_MOSI_GPIO_Port GPIOD
#define GPS_CS_Pin GPIO_PIN_7
#define GPS_CS_GPIO_Port GPIOD
#define GPS_SCK_Pin GPIO_PIN_3
#define GPS_SCK_GPIO_Port GPIOB
#define GPS_MISO_Pin GPIO_PIN_4
#define GPS_MISO_GPIO_Port GPIOB
#define GPS_EXTINT_Pin GPIO_PIN_5
#define GPS_EXTINT_GPIO_Port GPIOB
#define GPS_RESET_Pin GPIO_PIN_6
#define GPS_RESET_GPIO_Port GPIOB
#define TPU_TX_Pin GPIO_PIN_0
#define TPU_TX_GPIO_Port GPIOE
#define TPU_RX_Pin GPIO_PIN_1
#define TPU_RX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
/* Software entry into the ST ROM bootloader (USB DFU) without the BOOT/RESET buttons:
 * the USB console command 'B' stores DFU_MAGIC at DFU_MAGIC_ADDR and resets; the first thing
 * main() does is check it and jump to system memory. RAM_D1 (AXI SRAM), unused by the linker, keeps its contents across a software reset. */
#define DFU_MAGIC       0x41544844UL   /* "ATHD" */
#define DFU_MAGIC_ADDR  0x24000000UL
#define DFU_SYSMEM_ADDR 0x1FF09800UL   /* ST system memory (AN2606) */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

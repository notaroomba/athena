/**
  ******************************************************************************
  * @file    pd.c
  * @brief   TPS25751 USB-PD Controller Configuration Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pd.h"
#include "stm32g4xx_hal.h"

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Load firmware to TPS25751 via I2C
  * @param  hi2c: pointer to I2C handle
  * @param  firmware_data: pointer to firmware binary data
  * @param  firmware_size: size of firmware data in bytes
  * @param  device_addr: TPS25751 I2C 7-bit address (0x20 for address #1)
  * @retval HAL status
  */
HAL_StatusTypeDef TPS25751_LoadFirmware(I2C_HandleTypeDef *hi2c, 
                                        const char *firmware_data, 
                                        int firmware_size, 
                                        uint8_t device_addr)
{
    HAL_StatusTypeDef status;
    const int CHUNK_SIZE = 64; // I2C write chunk size
    int bytes_written = 0;
    
    // TPS25751 expects firmware to be written in chunks
    while (bytes_written < firmware_size)
    {
        int bytes_to_write = (firmware_size - bytes_written) > CHUNK_SIZE ? 
                             CHUNK_SIZE : (firmware_size - bytes_written);
        
        // Write chunk to device
        // HAL automatically shifts device_addr left by 1
        status = HAL_I2C_Master_Transmit(hi2c, 
                                         device_addr << 1, 
                                         (uint8_t*)&firmware_data[bytes_written], 
                                         bytes_to_write, 
                                         1000); // 1 second timeout
        
        if (status != HAL_OK)
        {
            return status;
        }
        
        bytes_written += bytes_to_write;
        
        // Small delay between chunks to allow processing
        HAL_Delay(10);
    }
    
    return HAL_OK;
}

/**
  * @brief  Write firmware to external EEPROM at address 0x50
  * @param  hi2c: pointer to I2C handle
  * @param  data: pointer to firmware binary data
  * @param  size: size of firmware data in bytes
  * @retval HAL status
  */
HAL_StatusTypeDef TPS25751_WriteEEPROM(I2C_HandleTypeDef *hi2c, 
                                       const char *data, 
                                       int size)
{
    HAL_StatusTypeDef status;
    const uint8_t EEPROM_ADDR = 0x50; // Fixed EEPROM address per datasheet
    const int PAGE_SIZE = 64; // Typical EEPROM page size
    uint16_t mem_addr = 0;
    int bytes_written = 0;
    
    while (bytes_written < size)
    {
        int bytes_to_write = (size - bytes_written) > PAGE_SIZE ? 
                             PAGE_SIZE : (size - bytes_written);
        
        // Write page to EEPROM
        status = HAL_I2C_Mem_Write(hi2c, 
                                   EEPROM_ADDR << 1, 
                                   mem_addr, 
                                   I2C_MEMADD_SIZE_16BIT,
                                   (uint8_t*)&data[bytes_written], 
                                   bytes_to_write, 
                                   1000);
        
        if (status != HAL_OK)
        {
            return status;
        }
        
        bytes_written += bytes_to_write;
        mem_addr += bytes_to_write;
        
        // EEPROM write cycle time (typically 5ms)
        HAL_Delay(5);
    }
    
    return HAL_OK;
}

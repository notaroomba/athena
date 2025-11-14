/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_bmp388_interface_template.c
 * @brief     driver bmp388 interface template source file
 * @version   2.0.0
 * @author    Shifeng Li
 * @date      2021-04-12
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/04/12  <td>2.0      <td>Shifeng Li  <td>format the code
 * <tr><td>2020/12/20  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_bmp388_interface.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <athena.h>

// External references from main.c
extern SPI_HandleTypeDef hspi4;

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t bmp388_interface_iic_init(void)
{
    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t bmp388_interface_iic_deinit(void)
{   
    return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bmp388_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t bmp388_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return 0;
}

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t bmp388_interface_spi_init(void)
{
    char debug_msg[64];
    sprintf(debug_msg, "BMP388: SPI init\r\n");
    CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t bmp388_interface_spi_deinit(void)
{
    char debug_msg[64];
    sprintf(debug_msg, "BMP388: SPI deinit\r\n");
    CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
    return 0;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bmp388_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint8_t reg_addr = reg | 0x80;  // Set MSB for read operation
    
    // Pull CS low
    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
    
    // Transmit register address
    status = HAL_SPI_Transmit(&hspi4, &reg_addr, 1, 100);
    if (status != HAL_OK) {
        print("BMP388: SPI transmit failed\r\n");
        HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
        return 1;
    }
    
    // Receive data
    status = HAL_SPI_Receive(&hspi4, buf, len, 100);
    
    // Pull CS high
    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
    
    if (status != HAL_OK) {
        char debug_msg[64];
        sprintf(debug_msg, "BMP388: SPI read failed (reg=0x%02X)\r\n", reg);
        CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
        return 1;
    }
    
    // Debug: Print chip ID read (register 0x00)
    if (reg == 0x00) {
        char debug_msg[64];
        sprintf(debug_msg, "BMP388: Chip ID read = 0x%02X (expected 0x50)\r\n", buf[0]);
        CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
    }
    
    return 0;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t bmp388_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint8_t reg_addr = reg & 0x7F;  // Clear MSB for write operation
    
    // Pull CS low
    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
    
    // Transmit register address
    status = HAL_SPI_Transmit(&hspi4, &reg_addr, 1, 100);
    if (status != HAL_OK) {
        HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
        return 1;
    }
    
    // Transmit data
    status = HAL_SPI_Transmit(&hspi4, buf, len, 100);
    
    // Pull CS high
    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
    
    if (status != HAL_OK) {
        char debug_msg[64];
        sprintf(debug_msg, "BMP388: SPI write failed (reg=0x%02X)\r\n", reg);
        CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
        return 1;
    }
    
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void bmp388_interface_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void bmp388_interface_debug_print(const char *const fmt, ...)
{
    char debug_buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(debug_buffer, sizeof(debug_buffer), fmt, args);
    va_end(args);
    
    CDC_Transmit_FS((uint8_t*)debug_buffer, strlen(debug_buffer));
}

/**
 * @brief     interface receive callback
 * @param[in] type interrupt type
 * @note      none
 */
void bmp388_interface_receive_callback(uint8_t type)
{
    switch (type)
    {
        case BMP388_INTERRUPT_STATUS_FIFO_WATERMARK :
        {
            bmp388_interface_debug_print("bmp388: irq fifo watermark.\n");
            
            break;
        }
        case BMP388_INTERRUPT_STATUS_FIFO_FULL :
        {
            bmp388_interface_debug_print("bmp388: irq fifo full.\n");
            
            break;
        }
        case BMP388_INTERRUPT_STATUS_DATA_READY :
        {
            bmp388_interface_debug_print("bmp388: irq data ready.\n");
            
            break;
        }
        default :
        {
            bmp388_interface_debug_print("bmp388: unknown code.\n");
            
            break;
        }
    }
}
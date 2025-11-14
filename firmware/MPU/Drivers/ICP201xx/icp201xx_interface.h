/**
 * Copyright (c) 2020 by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * @file      driver_icp201xx_interface.h
 * @brief     driver icp201xx interface header file
 * @version   1.0.0
 * @date      2025-11-14
 */

#ifndef DRIVER_ICP201XX_INTERFACE_H
#define DRIVER_ICP201XX_INTERFACE_H

#include "Icp201xx.h"
#include "Icp201xxDriver.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief ICP201xx device structure
 */
typedef struct {
    inv_icp201xx_t icp_device;
    uint8_t i2c_address;
    uint8_t spi_in_transaction;
    uint8_t use_spi;
} ICP201xx_t;

/**
 * @brief Initialize ICP201xx for I2C interface
 * @param[in] icp pointer to ICP201xx device structure
 * @param[in] lsb LSB of I2C address (0 or 1)
 */
void ICP201xx_init_i2c(ICP201xx_t *icp, uint8_t lsb);

/**
 * @brief Initialize ICP201xx for SPI interface
 * @param[in] icp pointer to ICP201xx device structure
 */
void ICP201xx_init_spi(ICP201xx_t *icp);

/**
 * @brief Begin communication with ICP201xx
 * @param[in] icp pointer to ICP201xx device structure
 * @return status code (0 = success, negative = error)
 */
int ICP201xx_begin(ICP201xx_t *icp);

/**
 * @brief Start continuous measurement
 * @param[in] icp pointer to ICP201xx device structure
 * @return status code (0 = success, negative = error)
 */
int ICP201xx_start(ICP201xx_t *icp);

/**
 * @brief Perform single measurement
 * @param[in] icp pointer to ICP201xx device structure
 * @param[out] pressure_kp pointer to store pressure in kPa
 * @param[out] temperature_C pointer to store temperature in Celsius
 * @return status code (0 = success, negative = error)
 */
int ICP201xx_singleMeasure(ICP201xx_t *icp, float *pressure_kp, float *temperature_C);

/**
 * @brief Get data from FIFO
 * @param[in] icp pointer to ICP201xx device structure
 * @param[out] pressure pointer to store pressure in kPa
 * @param[out] temperature pointer to store temperature in Celsius
 * @return status code (0 = success, negative = error)
 */
int ICP201xx_getData(ICP201xx_t *icp, float *pressure, float *temperature);

/**
 * @brief Warm up sensor (discard first 14 samples)
 * @param[in] icp pointer to ICP201xx device structure
 * @param[in] op_mode operating mode
 * @param[in] meas_mode measurement mode
 */
void ICP201xx_app_warmup(ICP201xx_t *icp, icp201xx_op_mode_t op_mode, icp201xx_meas_mode_t meas_mode);

/**
 * @brief Clear interrupt status
 * @param[in] icp pointer to ICP201xx device structure
 * @return interrupt status flags
 */
uint8_t ICP201xx_clear_interrupt_status(ICP201xx_t *icp);

/**
 * @defgroup icp201xx_interface_driver icp201xx interface driver function
 * @brief    icp201xx interface driver modules
 * @ingroup  icp201xx_driver
 * @{
 */

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t icp201xx_interface_iic_init(void);

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t icp201xx_interface_iic_deinit(void);

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
uint8_t icp201xx_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

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
uint8_t icp201xx_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t icp201xx_interface_spi_init(void);

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t icp201xx_interface_spi_deinit(void);

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
uint8_t icp201xx_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len);

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
uint8_t icp201xx_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void icp201xx_interface_delay_ms(uint32_t ms);

/**
 * @brief     interface delay us
 * @param[in] us time
 * @note      none
 */
void icp201xx_interface_delay_us(uint32_t us);

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void icp201xx_interface_debug_print(const char *const fmt, ...);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif

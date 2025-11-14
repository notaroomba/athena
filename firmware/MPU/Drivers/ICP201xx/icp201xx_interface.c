/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
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
 */
 
#include "main.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <athena.h>
#include "Icp201xx.h"
#include "Icp201xxDriver.h"

// External references from main.c
extern SPI_HandleTypeDef hspi4;

static int icp201xx_i2c_write(void * ctx, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int icp201xx_i2c_read(void * ctx, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static int icp201xx_spi_write(void * ctx, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int icp201xx_spi_read(void * ctx, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);

// I2C address configuration
#define ICP201XX_I2C_ADDRESS 0x63

typedef struct {
    inv_icp201xx_t icp_device;
    uint8_t i2c_address;
    uint8_t spi_in_transaction;
    uint8_t use_spi;
} ICP201xx_t;

// Forward declarations for functions used before definition
void ICP201xx_app_warmup(ICP201xx_t *icp, icp201xx_op_mode_t op_mode, icp201xx_meas_mode_t meas_mode);
int ICP201xx_getData(ICP201xx_t *icp, float *pressure, float *temperature);

// ICP201xx init for I2C interface
void ICP201xx_init_i2c(ICP201xx_t *icp, uint8_t lsb) {
    icp->i2c_address = ICP201XX_I2C_ADDRESS + (lsb ? 0x1 : 0);
    icp->use_spi = 0;
    icp->spi_in_transaction = 0;
}

// ICP201xx init for SPI interface
void ICP201xx_init_spi(ICP201xx_t *icp) {
    icp->use_spi = 1;
    icp->spi_in_transaction = 0;
    icp->i2c_address = 0;
}

/* starts communication with the ICP201xx */
int ICP201xx_begin(ICP201xx_t *icp) {
    inv_icp201xx_serif_t icp_serif;
    int rc = 0;
    uint8_t who_am_i;
    uint8_t icp_version;

    if (!icp->use_spi) {
        // I2C mode
        icp_serif.if_mode = ICP201XX_IF_I2C;
        icp_serif.read_reg  = icp201xx_i2c_read;
        icp_serif.write_reg = icp201xx_i2c_write;
    } else {
        // SPI mode
        HAL_GPIO_WritePin(ICP_CS_GPIO_Port, ICP_CS_Pin, GPIO_PIN_SET);
        icp_serif.if_mode = ICP201XX_IF_4_WIRE_SPI;
        icp_serif.read_reg  = icp201xx_spi_read;
        icp_serif.write_reg = icp201xx_spi_write;
    }
    
    /* Initialize serial interface between MCU and Icp201xx */
    icp_serif.context   = (void*)icp;
    icp_serif.max_read  = 2048; /* maximum number of bytes allowed per serial read */
    icp_serif.max_write = 2048; /* maximum number of bytes allowed per serial write */
    
    rc = inv_icp201xx_init(&icp->icp_device, &icp_serif);
    if (rc != INV_ERROR_SUCCESS) {
        return rc;
    }
    
    rc = inv_icp201xx_soft_reset(&icp->icp_device);
    if (rc != INV_ERROR_SUCCESS) {
        return rc;
    }
    
    /* Check WHOAMI */
    rc = inv_icp201xx_get_devid_version(&icp->icp_device, &who_am_i, &icp_version);
    if(rc != 0) {
        return -2;
    }
    if (who_am_i != EXPECTED_DEVICE_ID) {
        return -3;
    }
    
    /* Boot up OTP config */
    rc = inv_icp201xx_OTP_bootup_cfg(&icp->icp_device);
    if(rc != 0) {
        return rc;
    }
    
    // successful init, return 0
    return 0;
}

int ICP201xx_start(ICP201xx_t *icp) {
    int rc = 0;
    rc |= inv_icp201xx_soft_reset(&icp->icp_device);
    rc |= inv_icp201xx_config(&icp->icp_device, ICP201XX_OP_MODE0, ICP201XX_FIFO_READOUT_MODE_PRES_TEMP);
    ICP201xx_app_warmup(icp, ICP201XX_OP_MODE0, ICP201XX_MEAS_MODE_CONTINUOUS);
    return rc;
}

int ICP201xx_singleMeasure(ICP201xx_t *icp, float *pressure_kp, float *temperature_C)
{
    inv_icp201xx_trigger_meas(&icp->icp_device);
    HAL_Delay(20);
    return ICP201xx_getData(icp, pressure_kp, temperature_C);
}

int ICP201xx_getData(ICP201xx_t *icp, float *pressure, float *temperature) {
    uint8_t fifo_packets;
    uint8_t fifo_data[6];
    int32_t data_press, data_temp;
    
    /** Read measurements count in FIFO **/
    if (inv_icp201xx_get_fifo_count(&icp->icp_device, &fifo_packets))
        return -2;

    if (fifo_packets)
    {
        inv_icp201xx_get_fifo_data(&icp->icp_device, 1, fifo_data);
        if (!icp->use_spi) {
            /* Perform dummy read to 0x00 register as last transaction after FIFO read for I2C interface */
            do {
                uint8_t dummy_reg = 0;
                icp201xx_i2c_read(&icp->icp_device, 0, &dummy_reg, 1);
            } while(0);
        }
        
        inv_icp201xx_process_raw_data(&icp->icp_device, 1, fifo_data, &data_press, &data_temp);
        
        /** P = (POUT/2^17)*40kPa + 70kPa **/
        if (data_press & 0x080000)
            data_press |= 0xFFF00000;
        *pressure = ((float)(data_press) * 40 / 131072) + 70;
        *pressure = roundf(*pressure * 1000.0f) / 1000.0f;

        /* T = (TOUT/2^18)*65C + 25C */
        if (data_temp & 0x080000)
            data_temp |= 0xFFF00000;
        *temperature = ((float)(data_temp) * 65 / 262144) + 25;
        *temperature = roundf(*temperature * 10.0f) / 10.0f;

        return 0;
    }
    return -1;
}

static int icp201xx_i2c_write(void * ctx, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
    (void)ctx;
    (void)reg;
    (void)wbuffer;
    (void)wlen;
    // TODO: Implement I2C write
    return 0;
}

static int icp201xx_i2c_read(void * ctx, uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
    (void)ctx;
    (void)reg;
    (void)rbuffer;
    (void)rlen;
    // TODO: Implement I2C read
    return 0;
}

/*
 * spi_write might be called for writing read or write accesses.
 * First byte differenciate between those 0x33 => write, 0x3C => read
 */
static int icp201xx_spi_write(void * ctx, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
    (void)reg;
    ICP201xx_t* icp = (ICP201xx_t*)ctx;
    
    if(!icp->spi_in_transaction)
    {
        /* This is register address stage */
        HAL_GPIO_WritePin(ICP_CS_GPIO_Port, ICP_CS_Pin, GPIO_PIN_RESET);
        for(uint32_t i = 0; i < wlen; i++) {
            uint8_t data = wbuffer[i];
            HAL_SPI_Transmit(&hspi4, &data, 1, 1000);
        }
        icp->spi_in_transaction = 1;
    } else {
        // this is a data write => end the transaction
        for(uint32_t i = 0; i < wlen; i++) {
            uint8_t data = wbuffer[i];
            HAL_SPI_Transmit(&hspi4, &data, 1, 1000);
        }
        HAL_GPIO_WritePin(ICP_CS_GPIO_Port, ICP_CS_Pin, GPIO_PIN_SET);
        icp->spi_in_transaction = 0;
    }
    return 0;
}

static int icp201xx_spi_read(void * ctx, uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
    (void)reg;
    ICP201xx_t* icp = (ICP201xx_t*)ctx;
    
    // Read is always started by a write (0x3C command), but ensure CS is LOW
    if(!icp->spi_in_transaction) {
        HAL_GPIO_WritePin(ICP_CS_GPIO_Port, ICP_CS_Pin, GPIO_PIN_RESET);
    }
    
    for(uint32_t i = 0; i < rlen; i++) {
        HAL_SPI_Receive(&hspi4, &rbuffer[i], 1, 1000);
    }
    HAL_GPIO_WritePin(ICP_CS_GPIO_Port, ICP_CS_Pin, GPIO_PIN_SET);
    icp->spi_in_transaction = 0;
    return 0;
}

/* ICP201xx warm up. 
 * If FIR filter is enabled, it will cause a settling effect on the first 14 pressure values. 
 * Therefore the first 14 pressure output values are discarded.
 **/
void ICP201xx_app_warmup(ICP201xx_t *icp, icp201xx_op_mode_t op_mode, icp201xx_meas_mode_t meas_mode)
{
    (void)op_mode;
    (void)meas_mode;
    volatile uint8_t fifo_packets = 0;
    uint8_t fifo_packets_to_skip = 14;

    do {
        fifo_packets = 0;
        if (!inv_icp201xx_get_fifo_count(&icp->icp_device, (uint8_t*)&fifo_packets) && (fifo_packets >= fifo_packets_to_skip))
        {
            uint8_t i_status = 0;
            inv_icp201xx_flush_fifo(&icp->icp_device);

            inv_icp201xx_get_int_status(&icp->icp_device, &i_status);
            if (i_status)
                inv_icp201xx_clear_int_status(&icp->icp_device, i_status);
            break;
        }
        // delayMicroseconds(2) - using HAL_Delay with minimum 1ms
        HAL_Delay(1);
    } while (1);
}

uint8_t ICP201xx_clear_interrupt_status(ICP201xx_t *icp)
{
    uint8_t i_status = 0;
    inv_icp201xx_get_int_status(&icp->icp_device, &i_status);
    if (i_status)
        inv_icp201xx_clear_int_status(&icp->icp_device, i_status);
    return i_status;
}
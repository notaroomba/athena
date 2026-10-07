#include "imu_interface.h"
#include "main.h"
#include "inv_imu_driver.h"
#include "fusion.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

static inv_imu_device_t imu_dev[IMU_COUNT];

/* ---- raw SPI: ICM-45686 read = reg | 0x80, write = reg & 0x7F, CS active low ---- */

static int spi_read(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint8_t tx[33] = { 0 }, rx[33];
    if (len + 1 > sizeof(tx)) return -1;
    tx[0] = (uint8_t)(reg | 0x80);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(len + 1), 100);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    if (st != HAL_OK) return -1;
    memcpy(buf, &rx[1], len);
    return 0;
}

static int spi_write(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    uint8_t tx[33];
    if (len + 1 > sizeof(tx)) return -1;
    tx[0] = (uint8_t)(reg & 0x7F);
    memcpy(&tx[1], buf, len);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, tx, (uint16_t)(len + 1), 100);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    return st == HAL_OK ? 0 : -1;
}

/* The InvenSense transport callbacks carry no context pointer, so one tiny wrapper pair per chip. */
#define IMU_XPORT(n) \
    static int imu##n##_read (uint8_t r, uint8_t *b, uint32_t l)       { return spi_read (IMU##n##_CS_GPIO_Port, IMU##n##_CS_Pin, r, b, l); } \
    static int imu##n##_write(uint8_t r, const uint8_t *b, uint32_t l) { return spi_write(IMU##n##_CS_GPIO_Port, IMU##n##_CS_Pin, r, b, l); }
IMU_XPORT(1)
IMU_XPORT(2)
IMU_XPORT(3)

static void imu_sleep_us(uint32_t us) { HAL_Delay((us + 999) / 1000); }

static const struct { inv_imu_read_reg_t rd; inv_imu_write_reg_t wr; } xport[IMU_COUNT] = {
    { imu1_read, imu1_write }, { imu2_read, imu2_write }, { imu3_read, imu3_write },
};

static int imu_init_one(int idx)
{
    inv_imu_device_t *d = &imu_dev[idx];
    int rc; uint8_t whoami = 0;

    memset(d, 0, sizeof(*d));
    d->transport.read_reg   = xport[idx].rd;
    d->transport.write_reg  = xport[idx].wr;
    d->transport.serif_type = UI_SPI4;
    d->transport.sleep_us   = imu_sleep_us;

    drive_config0_t drv = { 0 };
    drv.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
    rc = inv_imu_write_reg(d, DRIVE_CONFIG0, 1, (uint8_t *)&drv);
    if (rc) return rc;
    imu_sleep_us(2);

    rc = inv_imu_get_who_am_i(d, &whoami);
    if (rc) return rc;
    if (whoami != INV_IMU_WHOAMI) {
        print("IMU%d: WHO_AM_I 0x%02X != 0x%02X\r\n", idx + 1, whoami, INV_IMU_WHOAMI);
        return -1;
    }
    rc  = inv_imu_soft_reset(d);
    rc |= inv_imu_set_accel_fsr(d, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G);
    rc |= inv_imu_set_gyro_fsr(d, GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS);
    rc |= inv_imu_set_accel_frequency(d, ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
    rc |= inv_imu_set_gyro_frequency(d, GYRO_CONFIG0_GYRO_ODR_400_HZ);
    rc |= inv_imu_set_accel_ln_bw(d, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
    rc |= inv_imu_set_gyro_ln_bw(d, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);
    rc |= inv_imu_set_accel_mode(d, PWR_MGMT0_ACCEL_MODE_LN);
    rc |= inv_imu_set_gyro_mode(d, PWR_MGMT0_GYRO_MODE_LN);
    return rc;
}

uint8_t IMU_Init(void)
{
    uint8_t mask = 0;
    HAL_Delay(3);                                   /* supply settle */
    for (int i = 0; i < IMU_COUNT; i++) {
        int rc = imu_init_one(i);
        print("IMU%d: %s\r\n", i + 1, rc ? "FAILED" : "ok");
        if (!rc) mask |= (uint8_t)(1u << i);
    }
    return mask;
}

int IMU_Read(int idx, IMU_Data *d)
{
    inv_imu_sensor_data_t raw;
    if (inv_imu_get_register_data(&imu_dev[idx], &raw)) return -1;
    if (raw.accel_data[0] == INVALID_VALUE_FIFO || raw.gyro_data[0] == INVALID_VALUE_FIFO) return -1;
    for (int i = 0; i < 3; i++) {
        d->accel_g[i]  = raw.accel_data[i] * (IMU_ACCEL_FSR_G / 32768.f);
        d->gyro_dps[i] = raw.gyro_data[i]  * (IMU_GYRO_FSR_DPS / 32768.f);
    }
    d->temperature_c = 25.f + raw.temp_data / 128.f;
    d->timestamp = GetTimestamp();
    return 0;
}

uint8_t IMU_ReadFused(uint8_t mask, IMU_Data *out)
{
    IMU_Data d[IMU_COUNT]; int ok[IMU_COUNT] = { 0 }; int n = 0; uint8_t used = 0;
    for (int i = 0; i < IMU_COUNT; i++) {
        if (!(mask & (1u << i))) continue;
        if (IMU_Read(i, &d[i]) == 0) { ok[i] = 1; n++; used |= (uint8_t)(1u << i); }
    }
    if (!n) return 0;
    memset(out, 0, sizeof(*out));
    if (n == 3) {                                   /* median rejects one wild unit */
        for (int a = 0; a < 3; a++) {
            out->accel_g[a]  = Fusion_Median3(d[0].accel_g[a],  d[1].accel_g[a],  d[2].accel_g[a]);
            out->gyro_dps[a] = Fusion_Median3(d[0].gyro_dps[a], d[1].gyro_dps[a], d[2].gyro_dps[a]);
        }
    } else {                                        /* mean of 2, or pass-through of 1 */
        for (int i = 0; i < IMU_COUNT; i++) if (ok[i])
            for (int a = 0; a < 3; a++) { out->accel_g[a] += d[i].accel_g[a] / n; out->gyro_dps[a] += d[i].gyro_dps[a] / n; }
    }
    for (int i = 0; i < IMU_COUNT; i++) if (ok[i]) { out->temperature_c += d[i].temperature_c / n; out->timestamp = d[i].timestamp; }
    return used;
}

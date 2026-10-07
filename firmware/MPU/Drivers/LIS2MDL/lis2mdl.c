#include "lis2mdl.h"
#include "lis2mdl_reg.h"
#include "main.h"
#include "athena.h"

extern SPI_HandleTypeDef hspi3;

/* SPI address byte per ST's reference example (lis2mdl_read_data_polling.c):
 * bit7 = read, bit6 = MS (address auto-increment for multi-byte access). */
#define SPI_READ  0x80
#define SPI_MS    0x40

/* CFG_REG_C bits (datasheet table 36): I2C_DIS | BDU | 4WSPI */
#define CFG_C_4WIRE_NO_I2C  (0x20 | 0x10 | 0x04)

static stmdev_ctx_t ctx;

static void cs(int on) { HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

/* One HAL call per transaction: the H7 HAL disables the SPI between calls and, unless
 * MasterKeepIOState is enabled, lets SCK float while CS is still low. Mode 3 (idle high)
 * would then see a bogus edge, so address and data always go out in a single transfer. */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    uint8_t tx[1 + 16];
    if (len > 16) return -1;
    tx[0] = (uint8_t)(reg | SPI_MS);
    for (uint16_t i = 0; i < len; i++) tx[1 + i] = bufp[i];
    cs(1);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(handle, tx, (uint16_t)(len + 1), 100);
    cs(0);
    return st == HAL_OK ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    uint8_t tx[1 + 16] = { 0 }, rx[1 + 16];
    if (len > 16) return -1;
    tx[0] = (uint8_t)(reg | SPI_READ | SPI_MS);
    cs(1);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(handle, tx, rx, (uint16_t)(len + 1), 100);
    cs(0);
    for (uint16_t i = 0; i < len; i++) bufp[i] = rx[1 + i];
    return st == HAL_OK ? 0 : -1;
}

static void platform_delay(uint32_t ms) { HAL_Delay(ms); }

int LIS2MDL_Init(void)
{
    uint8_t id = 0;
    ctx.write_reg = platform_write;
    ctx.read_reg  = platform_read;
    ctx.mdelay    = platform_delay;
    ctx.handle    = &hspi3;
    cs(0);

    /* Out of reset the part is in 3-wire SPI, so a read-modify-write (what the ST
     * driver does) would read garbage. A plain write works in either mode: force
     * 4-wire + I2C off first, then everything else can go through the driver. */
    uint8_t c = CFG_C_4WIRE_NO_I2C;
    if (platform_write(ctx.handle, LIS2MDL_CFG_REG_C, &c, 1)) return -2;
    HAL_Delay(1);

    if (lis2mdl_device_id_get(&ctx, &id)) return -2;
    if (id != LIS2MDL_ID) { print("LIS2MDL: WHO_AM_I 0x%02X != 0x%02X\r\n", id, LIS2MDL_ID); return -1; }

    /* No soft reset on purpose: SOFT_RST also clears CFG_REG_C, i.e. drops the part back
     * to 3-wire SPI, and the driver's reset-complete poll would then spin on garbage reads.
     * Power-on reset already put every register at its default; we write all of them below. */

    int32_t rc = 0;
    rc |= lis2mdl_block_data_update_set(&ctx, PROPERTY_ENABLE);
    rc |= lis2mdl_data_rate_set(&ctx, LIS2MDL_ODR_100Hz);
    rc |= lis2mdl_low_pass_bandwidth_set(&ctx, LIS2MDL_ODR_DIV_4);
    rc |= lis2mdl_set_rst_mode_set(&ctx, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);   /* hardware offset cancellation */
    rc |= lis2mdl_offset_temp_comp_set(&ctx, PROPERTY_ENABLE);
    rc |= lis2mdl_operating_mode_set(&ctx, LIS2MDL_CONTINUOUS_MODE);
    return rc ? -2 : 0;
}

int LIS2MDL_Read(float gauss[3])
{
    uint8_t rdy = 0; int16_t raw[3];
    if (lis2mdl_mag_data_ready_get(&ctx, &rdy)) return -1;
    if (!rdy) return 1;
    if (lis2mdl_magnetic_raw_get(&ctx, raw)) return -1;
    for (int a = 0; a < 3; a++) gauss[a] = lis2mdl_from_lsb_to_mgauss(raw[a]) * 0.001f;
    return 0;
}

int LIS2MDL_ReadTemp(float *deg_c)
{
    int16_t raw;
    if (lis2mdl_temperature_raw_get(&ctx, &raw)) return -1;
    *deg_c = lis2mdl_from_lsb_to_celsius(raw);
    return 0;
}

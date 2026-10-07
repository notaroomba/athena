#include "w25q.h"
#include "main.h"
#include <string.h>

extern QSPI_HandleTypeDef hqspi;

#define CMD_WREN        0x06
#define CMD_RDSR1       0x05
#define CMD_JEDEC_ID    0x9F
#define CMD_READ4       0x13
#define CMD_PP4         0x12
#define CMD_SE4         0x21
#define CMD_RST_EN      0x66
#define CMD_RST         0x99
#define SR1_BUSY        0x01

static int cmd(uint8_t instr, int has_addr, uint32_t addr, uint32_t nbytes, uint8_t *data, int write)
{
    QSPI_CommandTypeDef c;
    memset(&c, 0, sizeof c);
    c.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    c.Instruction       = instr;
    c.AddressMode       = has_addr ? QSPI_ADDRESS_1_LINE : QSPI_ADDRESS_NONE;
    c.AddressSize       = QSPI_ADDRESS_32_BITS;
    c.Address           = addr;
    c.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    c.DataMode          = nbytes ? QSPI_DATA_1_LINE : QSPI_DATA_NONE;
    c.NbData            = nbytes;
    c.DdrMode           = QSPI_DDR_MODE_DISABLE;
    c.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    c.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    if (HAL_QSPI_Command(&hqspi, &c, 1000) != HAL_OK) { HAL_QSPI_Abort(&hqspi); return -1; }
    if (nbytes) {
        if (write ? HAL_QSPI_Transmit(&hqspi, data, 1000) : HAL_QSPI_Receive(&hqspi, data, 1000)) { HAL_QSPI_Abort(&hqspi); return -1; }
    }
    return 0;                                                   /* Abort clears the BUSY state a timeout would otherwise leave behind */
}

static int wait_ready(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick(); uint8_t sr = SR1_BUSY;
    while (sr & SR1_BUSY) {
        if (cmd(CMD_RDSR1, 0, 0, 1, &sr, 0)) return -1;
        if (HAL_GetTick() - t0 > timeout_ms) return -1;
    }
    return 0;
}

int W25Q_Init(uint32_t *jedec_id)
{
    uint8_t id[3] = { 0 };
    wait_ready(500);                                            /* an erase may still be running from before an MCU reset */
    if (cmd(CMD_RST_EN, 0, 0, 0, NULL, 0) || cmd(CMD_RST, 0, 0, 0, NULL, 0)) return -1;
    HAL_Delay(1);                                               /* tRST 30 us */
    if (cmd(CMD_JEDEC_ID, 0, 0, 3, id, 0)) return -1;
    uint32_t v = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
    if (jedec_id) *jedec_id = v;
    return v == W25Q_JEDEC_ID ? 0 : -2;
}

int W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    return cmd(CMD_READ4, 1, addr, len, buf, 0);
}

int W25Q_Program(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (len == 0 || len > W25Q_PAGE || (addr % W25Q_PAGE) + len > W25Q_PAGE) return -1;
    if (cmd(CMD_WREN, 0, 0, 0, NULL, 0)) return -1;
    if (cmd(CMD_PP4, 1, addr, len, (uint8_t *)buf, 1)) return -1;
    return wait_ready(10);                                      /* tPP 3 ms max */
}

int W25Q_EraseSectorStart(uint32_t addr)                      /* issue the erase, do not wait: poll W25Q_Busy() */
{
    if (cmd(CMD_WREN, 0, 0, 0, NULL, 0)) return -1;
    return cmd(CMD_SE4, 1, addr & ~(W25Q_SECTOR - 1), 0, NULL, 0);
}

int W25Q_Busy(void)
{
    uint8_t sr = SR1_BUSY;
    if (cmd(CMD_RDSR1, 0, 0, 1, &sr, 0)) return 1;
    return (sr & SR1_BUSY) != 0;
}

int W25Q_EraseSector(uint32_t addr)
{
    if (cmd(CMD_WREN, 0, 0, 0, NULL, 0)) return -1;
    if (cmd(CMD_SE4, 1, addr & ~(W25Q_SECTOR - 1), 0, NULL, 0)) return -1;
    return wait_ready(500);                                     /* tSE 400 ms max */
}

int W25Q_IsErased(uint32_t addr, uint32_t len)
{
    uint8_t b[64];
    while (len) {
        uint32_t n = len > sizeof b ? sizeof b : len;
        if (W25Q_Read(addr, b, n)) return 0;
        for (uint32_t i = 0; i < n; i++) if (b[i] != 0xFF) return 0;
        addr += n; len -= n;
    }
    return 1;
}

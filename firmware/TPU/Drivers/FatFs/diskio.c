/*
 * diskio.c - FatFs glue for the microSD on SDMMC1 (TPU), polled HAL_SD transfers.
 * HAL_SD_Init/DeInit are owned by logger.c (the card is removable); every call here has a
 * timeout so a card pulled mid-transfer costs at most SD_TIMEOUT_MS, never a hang.
 */
#include "diskio.h"
#include "main.h"

extern SD_HandleTypeDef hsd1;
#define SD_TIMEOUT_MS 1000u

static int sd_wait_transfer(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
        if (HAL_GetTick() - t0 > timeout_ms) return 0;
    return 1;
}

DSTATUS disk_status(BYTE pdrv)
{
    (void)pdrv;
    return (hsd1.State == HAL_SD_STATE_READY) ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv) { return disk_status(pdrv); }

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    (void)pdrv;
    if (hsd1.State != HAL_SD_STATE_READY) return RES_NOTRDY;
    if (HAL_SD_ReadBlocks(&hsd1, buff, sector, count, SD_TIMEOUT_MS) != HAL_OK) return RES_ERROR;
    return sd_wait_transfer(SD_TIMEOUT_MS) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    (void)pdrv;
    if (hsd1.State != HAL_SD_STATE_READY) return RES_NOTRDY;
    if (HAL_SD_WriteBlocks(&hsd1, (uint8_t *)buff, sector, count, SD_TIMEOUT_MS) != HAL_OK) return RES_ERROR;
    return sd_wait_transfer(SD_TIMEOUT_MS) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)pdrv;
    HAL_SD_CardInfoTypeDef ci;
    switch (cmd) {
    case CTRL_SYNC:        return sd_wait_transfer(SD_TIMEOUT_MS) ? RES_OK : RES_ERROR;
    case GET_SECTOR_COUNT: if (HAL_SD_GetCardInfo(&hsd1, &ci) != HAL_OK) return RES_ERROR; *(DWORD *)buff = ci.LogBlockNbr; return RES_OK;
    case GET_SECTOR_SIZE:  *(WORD *)buff = 512; return RES_OK;
    case GET_BLOCK_SIZE:   *(DWORD *)buff = 1; return RES_OK;
    default:               return RES_PARERR;
    }
}

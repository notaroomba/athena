#include "logger.h"
#include "main.h"
#include "athena.h"
#include "athena_link.h"
#include "ff.h"
#include "w25q.h"
#include <stdio.h>
#include <string.h>

extern SD_HandleTypeDef hsd1;
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/* ------------------------------------------------------------------ SD card */
enum { SD_ABSENT = 0, SD_MOUNTED };
static FATFS   fs;
static FIL     fil;
static char    fname[16];
static uint8_t sd_state, cd_now, cd_stable, cd_learned, cd_present_level, sd_nofs;
static uint32_t cd_change_ms, sd_retry_ms, sd_sync_ms, sd_flush_ms, sd_bytes, sd_errors, sd_mounts;
static uint8_t  sdbuf[8192];
static uint32_t sdbuf_len, sd_dropped;

/* ------------------------------------------------------------------ flash ring */
static int      fl_ok;
static uint32_t fl_id, fl_wp, fl_bytes, fl_errors, fl_flush_ms;
static uint8_t  flq[4096];                 /* ring of bytes waiting for the flash */
static uint32_t flq_head, flq_tail, fl_dropped;
static uint8_t  fl_sector_prepped;

static volatile uint8_t cmd_pending;       /* set from the USB RX interrupt, handled in Task */

/* ------------------------------------------------------------------ helpers */
static int cd_raw_present(void)
{
    /* pull-up on the board: the socket's detect switch pulls the pin low when a card is in.
     * If a mount ever succeeds, the level seen at that moment is remembered instead, so a
     * normally-closed socket works too. */
    int level = HAL_GPIO_ReadPin(SD_CD_GPIO_Port, SD_CD_Pin) == GPIO_PIN_SET;
    return cd_learned ? (level == cd_present_level) : (level == 0);
}

static void sd_unmount(const char *why)
{
    if (sd_state == SD_MOUNTED) print("sd: %s, closing %s (%lu bytes)\r\n", why, fname, (unsigned long)sd_bytes);
    f_close(&fil);                          /* may fail if the card is gone: harmless */
    f_mount(NULL, "", 0);
    if (hsd1.State != HAL_SD_STATE_RESET) HAL_SD_DeInit(&hsd1);   /* never touch SDMMC registers before its clock exists */
    sd_state = SD_ABSENT;
    sdbuf_len = 0;
    sd_retry_ms = HAL_GetTick();
}

static int sd_try_mount(void)
{
    print("sd: init (cd pin=%d)...\r\n", HAL_GPIO_ReadPin(SD_CD_GPIO_Port, SD_CD_Pin));
    hsd1.Instance = SDMMC1;                 /* same values CubeMX generates, owned here because the card is optional */
    hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv = 10;                /* 50 MHz / (2*10) = 2.5 MHz: the polled HAL write path under-ran at 12.5 MHz; 1.25 MB/s is plenty */
    /* HAL_SD_DeInit() on a never-initialised handle writes SDMMC registers with the peripheral
     * clock still gated, which hard-faults on the H7: only de-init something that was inited. */
    if (hsd1.State != HAL_SD_STATE_RESET) HAL_SD_DeInit(&hsd1);
    if (HAL_SD_Init(&hsd1) != HAL_OK) { print("sd: HAL_SD_Init failed (err 0x%lX)\r\n", (unsigned long)hsd1.ErrorCode); HAL_SD_DeInit(&hsd1); return -1; }
    print("sd: card init ok, mounting...\r\n");
    FRESULT fr = f_mount(&fs, "", 1);
    if (fr == FR_NO_FILESYSTEM) {
        if (!sd_nofs) print("sd: card has no FAT/exFAT filesystem. Send 'F' on this console to format it (erases the card).\r\n");
        sd_nofs = 1; HAL_SD_DeInit(&hsd1); return -2;
    }
    if (fr != FR_OK) { print("sd: mount failed (FatFs error %d)\r\n", fr); HAL_SD_DeInit(&hsd1); return -2; }
    sd_nofs = 0;

    /* next free ATHnnnnn.BIN */
    DIR dir; FILINFO fno; unsigned maxn = 0;
    if (f_opendir(&dir, "") != FR_OK) { print("sd: root dir unreadable\r\n"); f_mount(NULL, "", 0); HAL_SD_DeInit(&hsd1); return -3; }
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
        unsigned n; if (sscanf(fno.fname, "ATH%u.BIN", &n) == 1 && n > maxn) maxn = n;
    }
    f_closedir(&dir);
    FRESULT fo = FR_EXIST;
    for (int tries = 0; tries < 100 && fo == FR_EXIST; tries++) {          /* CREATE_NEW: never truncate an older log */
        snprintf(fname, sizeof fname, "ATH%05u.BIN", ++maxn);
        fo = f_open(&fil, fname, FA_WRITE | FA_CREATE_NEW);
    }
    if (fo != FR_OK) { print("sd: cannot create log file (%d)\r\n", fo); f_mount(NULL, "", 0); HAL_SD_DeInit(&hsd1); return -3; }
    print("sd: mounted, logging to %s\r\n", fname);
    f_sync(&fil);                           /* directory entry exists even if the card is yanked right away */
    sd_state = SD_MOUNTED; sd_bytes = 0; sd_mounts++;
    cd_present_level = HAL_GPIO_ReadPin(SD_CD_GPIO_Port, SD_CD_Pin) == GPIO_PIN_SET; cd_learned = 1;
    sd_sync_ms = sd_flush_ms = HAL_GetTick();
    HAL_SD_CardInfoTypeDef ci; HAL_SD_GetCardInfo(&hsd1, &ci);
    sd_dropped = 0;                                    /* drops before this mount do not count against the new file */
    print("sd: mounted %lu MB card, logging to %s\r\n", (unsigned long)(ci.LogBlockNbr / 2048u), fname);
    return 0;
}

static void sd_flush(int sync)
{
    if (sd_state != SD_MOUNTED) return;
    if (sdbuf_len) {
        UINT bw = 0;
        FRESULT fr = f_write(&fil, sdbuf, sdbuf_len, &bw);
        if (fr != FR_OK || bw != sdbuf_len) { sd_errors++; sd_unmount("write error"); return; }
        sd_bytes += bw; sdbuf_len = 0; sd_flush_ms = HAL_GetTick();
    }
    if (sync) {
        if (f_sync(&fil) != FR_OK) { sd_errors++; sd_unmount("sync error"); return; }
        sd_sync_ms = HAL_GetTick();
    }
}

static void sd_task(uint32_t now)
{
    /* debounced card detect */
    uint8_t raw = (uint8_t)cd_raw_present();
    if (raw != cd_now) { cd_now = raw; cd_change_ms = now; }
    if (cd_now != cd_stable && now - cd_change_ms > 100u) {
        cd_stable = cd_now;
        if (!cd_stable && sd_state == SD_MOUNTED) sd_unmount("card removed");
        if (cd_stable && sd_state == SD_ABSENT) sd_retry_ms = now - 10000u;   /* mount promptly */
        sd_nofs = 0;                                                        /* a different card may be fine */
    }
    if (sd_state == SD_ABSENT) {
        /* retry every 2 s while the detect line says present (or until the polarity is learned) */
        if ((cd_stable || !cd_learned) && now - sd_retry_ms >= (sd_nofs ? 15000u : 2000u)) {
            sd_retry_ms = now;
            if (sd_try_mount() == 0) cd_stable = cd_now = 1;
        }
        return;
    }
    if (sdbuf_len >= 2048u || (sdbuf_len && now - sd_flush_ms >= 500u)) sd_flush(0);
    if (now - sd_sync_ms >= 1000u) sd_flush(1);
}

/* Format the card (FAT32 or exFAT, picked by FatFs from the size). Erases everything on it:
 * only ever triggered by an explicit 'F' on the USB console. */
static void sd_format(void)
{
    static uint8_t work[4096];
    if (sd_state == SD_MOUNTED) sd_unmount("format requested");
    print("sd: formatting...\r\n");
    if (hsd1.State != HAL_SD_STATE_RESET) HAL_SD_DeInit(&hsd1);
    hsd1.Instance = SDMMC1; hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING; hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B; hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE; hsd1.Init.ClockDiv = 10;
    if (HAL_SD_Init(&hsd1) != HAL_OK) { print("sd: no card\r\n"); return; }
    FRESULT fr = f_mkfs("", FM_ANY, 0, work, sizeof work);
    HAL_SD_DeInit(&hsd1);
    print("sd: format %s (%d)\r\n", fr == FR_OK ? "done" : "FAILED", fr);
    sd_nofs = 0; sd_retry_ms = HAL_GetTick() - 10000u;              /* mount it right away */
}

/* ------------------------------------------------------------------ flash ring */
static uint32_t flq_count(void) { return (flq_head - flq_tail) % sizeof flq; }

static void fl_find_write_pointer(void)
{
    /* The writer keeps the sector after the write pointer erased, so scanning for the first
     * erased sector finds the end of the log even after the ring has wrapped. */
    uint32_t nsec = W25Q_SIZE / W25Q_SECTOR, s;
    for (s = 0; s < nsec; s++) if (W25Q_IsErased(s * W25Q_SECTOR, 16)) break;
    if (s == nsec) { fl_wp = 0; W25Q_EraseSector(0); W25Q_EraseSector(W25Q_SECTOR); return; }   /* no boundary: start over */
    if (s == 0 && W25Q_IsErased((nsec - 1) * W25Q_SECTOR, 16)) { fl_wp = 0; return; }   /* empty chip */
    /* previous sector (wrapping) holds the tail: find the last non-erased byte in it */
    uint32_t base = ((s + nsec - 1) % nsec) * W25Q_SECTOR, pos = 0; uint8_t b[64];
    for (uint32_t off = 0; off < W25Q_SECTOR; off += sizeof b) {
        if (W25Q_Read(base + off, b, sizeof b)) break;
        for (uint32_t i = 0; i < sizeof b; i++) if (b[i] != 0xFF) pos = off + i + 1;
    }
    fl_wp = (base + pos) % W25Q_SIZE;
}

static void fl_task(uint32_t now)
{
    if (!fl_ok) return;
    uint32_t n = flq_count();
    if (!n) return;
    if (n < W25Q_PAGE && now - fl_flush_ms < 200u) return;      /* coalesce into page-sized programs */
    if (W25Q_Busy()) return;                                     /* a look-ahead erase is still running: bytes keep queueing */
    if (fl_wp % W25Q_SECTOR == 0 && !fl_sector_prepped) {        /* entering a fresh sector (erased earlier as the boundary) */
        if (!W25Q_IsErased(fl_wp, 64) && W25Q_EraseSector(fl_wp)) { fl_errors++; return; }   /* cold start / corrupted only */
        if (W25Q_EraseSectorStart((fl_wp + W25Q_SECTOR) % W25Q_SIZE)) { fl_errors++; return; } /* next boundary, in the background */
        fl_sector_prepped = 1;
        return;                                                  /* come back when the erase has finished */
    }
    uint32_t room = W25Q_PAGE - (fl_wp % W25Q_PAGE);
    if (n > room) n = room;
    uint8_t page[W25Q_PAGE];
    for (uint32_t i = 0; i < n; i++) page[i] = flq[(flq_tail + i) % sizeof flq];
    if (W25Q_Program(fl_wp, page, n)) { fl_errors++; fl_flush_ms = now; return; }
    flq_tail = (flq_tail + n) % sizeof flq; fl_wp = (fl_wp + n) % W25Q_SIZE; fl_bytes += n; fl_flush_ms = now;
    if (fl_wp % W25Q_SECTOR == 0) fl_sector_prepped = 0;
}

static int usb_send_blocking(const uint8_t *d, uint16_t n)      /* 0 ok, -1 host stalled */
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 1000u) if (CDC_Transmit_FS((uint8_t *)d, n) != 1) return 0;
    return -1;
}

static void fl_dump(void)
{
    /* raw frame stream from 0 to the write pointer; data from before a wrap is not included */
    char hdr[48]; int hn = snprintf(hdr, sizeof hdr, "\r\nLOGDUMP %lu\r\n", (unsigned long)fl_wp);
    usb_send_blocking((uint8_t *)hdr, (uint16_t)hn);
    static uint8_t chunk[2][1024]; int k = 0;      /* CDC_Transmit_FS returns before the data has left: alternate buffers */
    for (uint32_t a = 0; a < fl_wp; a += sizeof chunk[0], k ^= 1) {
        uint32_t n = fl_wp - a > sizeof chunk[0] ? sizeof chunk[0] : fl_wp - a;
        if (W25Q_Read(a, chunk[k], n)) break;
        if (usb_send_blocking(chunk[k], (uint16_t)n)) { print("\r\nLOGABORT host stalled\r\n"); return; }
    }
    HAL_Delay(5);
    usb_send_blocking((uint8_t *)"\r\nLOGEND\r\n", 10);
}

/* ------------------------------------------------------------------ public */
void Logger_Init(void)
{
    int rc = W25Q_Init(&fl_id);
    fl_ok = (rc == 0);
    print("flash: W25Q_Init rc=%d id=%06lX\r\n", rc, (unsigned long)fl_id);
    if (fl_ok) { fl_find_write_pointer(); print("flash: W25Q256 ok, log resumes at %lu KB\r\n", (unsigned long)(fl_wp / 1024)); }
    cd_now = cd_stable = (uint8_t)cd_raw_present();
    sd_retry_ms = HAL_GetTick() - 10000u;                        /* first attempt right away in Task() */
    uint8_t tmp[64]; size_t n = Link_Encode(tmp, LINK_PKT_TEXT, "Athena TPU boot", 15);
    Logger_Write(tmp, (uint32_t)n);
}

void Logger_Write(const uint8_t *data, uint32_t len)
{
    if (sd_state == SD_MOUNTED) {
        if (sdbuf_len + len > sizeof sdbuf) sd_flush(0);         /* a burst after a blocking step (format, LoRa, erase) fills it: write now instead of dropping */
        if (sdbuf_len + len <= sizeof sdbuf) { memcpy(sdbuf + sdbuf_len, data, len); sdbuf_len += len; }
        else sd_dropped++;
    }
    if (fl_ok) {
        if (flq_count() + len < sizeof flq) { for (uint32_t i = 0; i < len; i++) flq[(flq_head + i) % sizeof flq] = data[i]; flq_head = (flq_head + len) % sizeof flq; }
        else fl_dropped++;
    }
}

void Logger_Task(void)
{
    uint32_t now = HAL_GetTick();
    sd_task(now);
    fl_task(now);
    if (cmd_pending) {
        uint8_t c = cmd_pending; cmd_pending = 0;
        if (c == 'D') { sd_flush(1); fl_dump(); }
        else if (c == 'E' && fl_ok) { fl_wp = 0; flq_head = flq_tail = 0; W25Q_EraseSector(0); W25Q_EraseSector(W25Q_SECTOR); print("flash: log restarted\r\n"); }
        else if (c == 'S') { if (sd_state == SD_MOUNTED) { sd_flush(1); print("sd: synced %s (%lu bytes)\r\n", fname, (unsigned long)sd_bytes); } else print("sd: no card mounted\r\n"); }
        else if (c == 'F') sd_format();
    }
}

void Logger_StatusLine(char *out, size_t n)
{
    if (sd_state == SD_MOUNTED) snprintf(out, n, "sd=%s %luKB%s", fname, (unsigned long)(sd_bytes / 1024), sd_dropped ? "!" : "");
    else snprintf(out, n, "sd=%s", cd_stable ? "mounting" : "none");
    size_t l = strlen(out);
    if (fl_ok) snprintf(out + l, n - l, " flash=%lu.%luMB%s", (unsigned long)(fl_wp >> 20), (unsigned long)((fl_wp & 0xFFFFF) * 10 >> 20), fl_errors ? "!" : "");
    else snprintf(out + l, n - l, " flash=none");
}

void Logger_UsbRx(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) if (buf[i] == 'D' || buf[i] == 'E' || buf[i] == 'S' || buf[i] == 'F') cmd_pending = buf[i];
}

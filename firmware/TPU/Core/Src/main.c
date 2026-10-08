/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "athena.h"
#include "athena_link.h"
#include "sx127x.h"
#include "ublox.h"
#include "logger.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LORA_FREQ_HZ    433000000u   // RA-02 band 410-525 MHz; match the ground station
#define LORA_TX_DBM     17
#define TELEM_RATE_HZ   2            // ~45 B frame at SF7/125k is ~90 ms of air time
#define STATUS_PRINT_MS 500
#define LED_IDENTITY_MS 3000         // show the MCU identity colour this long after reset
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan2;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint32_t dfu_boot_magic, dfu_boot_bkp;
static uint8_t  leds_off;                      // 'L' on the USB console: dark board (night, bench)
static uint32_t fault_rec[6];                 // crash record from the previous run (magic, pc, lr, cfsr, hfsr, bfar)
static Link             mpu_link;        // UART8 <-> MPU
static Link             air_link;        // frames received over LoRa (ground -> rocket)
static uint8_t          uart8_rx_byte;
static uint8_t          uart_frame[sizeof(Athena_GpsFix) + LINK_OVERHEAD];   // owned by the UART8 IT transfer
static uint8_t          usb_frame[sizeof(Athena_GpsFix) + LINK_OVERHEAD];
static uint8_t          lora_frame[sizeof(Athena_Telemetry) + LINK_OVERHEAD];
static Athena_State     mpu_state;
static uint32_t         mpu_state_ms, state_count;
static Athena_GpsFix    gps;
static uint32_t         gps_ms, gps_count;
static uint8_t          lora_ok, gps_cfg_failed;
static int16_t          last_rssi; static int8_t last_snr; static uint32_t air_count;
static uint8_t          lora_fail;
static uint8_t          uart7_rx_byte;                       // DA14531 (CodeLess AT) console
static char             bt_line[96]; static uint8_t bt_len; static uint32_t bt_rx_bytes;
static Link             bt_link;         // DA14531 UART: command frames from a phone (DSPS) + AT text (CodeLess)
static uint32_t         gps_rate_count, gps_rate_ms, gps_rate_x10;   /* measured fix rate, Hz*10 */
static Link             usb_link;        // USB console: command frames from the dashboard + single-character commands
static Athena_SpuStatus spu;             // latest SPU status (relayed by the MPU)
static uint8_t          spu_frame[sizeof(Athena_SpuStatus) + LINK_OVERHEAD];   // kept for USB + the radio
static uint32_t         spu_ms, spu_count, last_spu_air_ms;
static uint8_t          cmd_frame[sizeof(Athena_Cmd) + LINK_OVERHEAD], cmd_pending[sizeof(Athena_Cmd) + LINK_OVERHEAD];
static uint8_t          cmd_pending_len; // command waiting for UART8 (-> MPU -> SPU)
static uint32_t         cmd_count;
static uint8_t          bt_frame[sizeof(Athena_SpuStatus) + LINK_OVERHEAD];    // owned by the UART7 IT transfer (-> DA14531 -> phone/laptop)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */
static void Athena_DfuPoll(void);
void Athena_DfuRequest(uint8_t c);
static void on_mpu_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user);
static void on_air_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user);
static void on_usb_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user);
static void on_usb_text(uint8_t b, void *user);
static void on_bt_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user);
static void on_bt_text(uint8_t b, void *user);
static void forward_cmd(const uint8_t *payload, uint8_t len, const char *src);
void Athena_UsbRx(const uint8_t *buf, uint32_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  SCB->VTOR = FLASH_BASE;                                       /* after a DFU "leave" the ROM jumps here without a reset and VTOR still points at its own table */
  __DSB(); __ISB();
  dfu_boot_magic = *(volatile uint32_t *)DFU_MAGIC_ADDR;        /* printed later: tells whether the word survived the reset */
  DFU_BKP_ENABLE();
  dfu_boot_bkp = DFU_BKP_REG;
  if (dfu_boot_magic == DFU_MAGIC || dfu_boot_bkp == DFU_MAGIC) {   /* 'B' on the USB console asked for DFU */
    *(volatile uint32_t *)DFU_MAGIC_ADDR = 0;
    DFU_BKP_REG = 0;
    SysTick->CTRL = 0;
    SCB->VTOR = DFU_SYSMEM_ADDR;                                /* ROM vector table; interrupts stay enabled as after a real reset */
    __set_MSP(*(volatile uint32_t *)DFU_SYSMEM_ADDR);
    ((void (*)(void))(*(volatile uint32_t *)(DFU_SYSMEM_ADDR + 4)))();   /* never returns */
  }

  { volatile uint32_t *r = (volatile uint32_t *)(DFU_MAGIC_ADDR + 0x10);   /* left by Athena_FaultSave() / Error_Handler() before their reset */
    if (r[0] == 0x46415554UL || r[0] == 0x4552524FUL) { for (int i = 0; i < 6; i++) fault_rec[i] = r[i]; r[0] = 0; } }
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  Athena_LED_PinConfig led_pins = {
      .port_r = TPU_R_GPIO_Port,
      .pin_r = TPU_R_Pin,
      .port_g = TPU_G_GPIO_Port,
      .pin_g = TPU_G_Pin,
      .port_b = TPU_B_GPIO_Port,
      .pin_b = TPU_B_Pin
  };
  Athena_Init(&led_pins, NULL);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_QUADSPI_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_FDCAN2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_SDMMC1_SD_Init();
  /* USER CODE BEGIN 2 */
  Set_LED_Color(LED_RED);                              // identity colour: TPU = red (MPU green, SPU blue)
  Link_Init(&mpu_link, on_mpu_packet, NULL);
  Link_Init(&air_link, on_air_packet, NULL);
  Link_Init(&usb_link, on_usb_packet, NULL);
  usb_link.on_text = on_usb_text;                      // 'B'/'J' DFU, 'D'/'E'/'S'/'F' logger
  Link_Init(&bt_link, on_bt_packet, NULL);
  bt_link.on_text = on_bt_text;                        // CodeLess replies collected into lines
  HAL_UART_Receive_IT(&huart8, &uart8_rx_byte, 1);
  HAL_UART_Receive_IT(&huart7, &uart7_rx_byte, 1);     // DA14531 CodeLess replies (needs module J5/P0_6 -> PE7, see INTEGRATION.md)
  HAL_Delay(1000);                                     // USB CDC enumeration

  print("\r\n=== Athena TPU ===\r\n");
  print("dfu: boot magic sram=0x%08lX bkp=0x%08lX\r\n", (unsigned long)dfu_boot_magic, (unsigned long)dfu_boot_bkp);
  if (fault_rec[0]) print("%s before this reset: pc=0x%08lX lr=0x%08lX cfsr=0x%08lX hfsr=0x%08lX bfar=0x%08lX\r\n",
                          fault_rec[0] == 0x46415554UL ? "FAULT" : "ERROR_HANDLER", (unsigned long)fault_rec[1], (unsigned long)fault_rec[2],
                          (unsigned long)fault_rec[3], (unsigned long)fault_rec[4], (unsigned long)fault_rec[5]);
  lora_ok = (SX127x_Init(LORA_FREQ_HZ, LORA_TX_DBM) == 0);
  print("SX1278 %s (RegVersion 0x%02X)\r\n", lora_ok ? "ok" : "FAILED", SX127x_ReadReg(0x42));
  gps_cfg_failed = (uint8_t)Ublox_Init();
  print("NEO-M8U: %u config messages not ACKed (ack %lu nak %lu)\r\n", gps_cfg_failed, (unsigned long)Ublox_AckCount(), (unsigned long)Ublox_NakCount());
  print("init: logger\r\n");
  Logger_Init();                                       // W25Q256 ring log + microSD file (mounted when a card is present)
  print("init: done, entering loop\r\n");
  HAL_Delay(1500);                                     // DA14531 boots CodeLess from its flash (RST/P0_0 left floating)
  HAL_UART_Transmit(&huart7, (uint8_t *)"AT\r\n", 4, 100);
  print("DA14531: AT sent on UART7 @57600 (replies only visible once module J5/P0_6 is wired to PE7)\r\n");

  uint32_t last_telem_ms = 0, last_print_ms = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    Athena_DfuPoll();

    /* 0. logging back-ends: card detect, buffered writes, periodic sync, console commands */
    Logger_Task();

    /* 1. navigation state + SPU status arriving from the MPU (on_mpu_packet), USB console (on_usb_*) */
    Link_Process(&mpu_link);
    Link_Process(&usb_link);
    Link_Process(&bt_link);
    if (cmd_pending_len && huart8.gState == HAL_UART_STATE_READY) {   // command (uplink or USB) -> MPU -> SPU
      memcpy(cmd_frame, cmd_pending, cmd_pending_len);
      HAL_UART_Transmit_IT(&huart8, cmd_frame, cmd_pending_len); cmd_pending_len = 0;
    }

    /* 2. GPS: forward every NAV-PVT to the MPU, where the Kalman filter uses it */
    if (Ublox_Poll(&gps)) {
      gps_ms = now; gps_count++; gps_rate_count++;
      if ((now - gps_rate_ms) >= 5000u) {              // fix rate actually delivered by the receiver
        gps_rate_x10 = gps_rate_count * 10000u / (now - gps_rate_ms);
        gps_rate_count = 0; gps_rate_ms = now;
      }
      size_t n = Link_Encode(usb_frame, LINK_PKT_GPS, &gps, sizeof gps);
      Logger_Write(usb_frame, (uint32_t)n);
      CDC_Transmit_FS(usb_frame, (uint16_t)n);                 // to the USB dashboard (dropped if busy)
      if (huart8.gState == HAL_UART_STATE_READY) {
        memcpy(uart_frame, usb_frame, n);
        HAL_UART_Transmit_IT(&huart8, uart_frame, (uint16_t)n);
      }
    }

    /* 3. telemetry downlink: fused estimate + raw fix quality */
    if (lora_ok && (now - last_telem_ms) >= (1000u / TELEM_RATE_HZ)) {
      last_telem_ms = now;
      Athena_Telemetry t;
      Link_MakeTelemetry(&t, &mpu_state, (now - gps_ms) < 2000u ? &gps : NULL, now);
      size_t n = Link_Encode(lora_frame, LINK_PKT_TELEM, &t, sizeof t);
      Logger_Write(lora_frame, (uint32_t)n);
      CDC_Transmit_FS(lora_frame, (uint16_t)n);                // to the USB dashboard too
      if (huart7.gState == HAL_UART_STATE_READY) {             // and to the Bluetooth module (transparent with the DSPS firmware)
        memcpy(bt_frame, lora_frame, n);
        HAL_UART_Transmit_IT(&huart7, bt_frame, (uint16_t)n);
      }
      if (SX127x_Send(lora_frame, (uint8_t)n) == 0) {  // blocks ~90 ms, then returns to RX
        lora_fail = 0;
      } else if (++lora_fail >= 2) {                   // no TxDone twice: reset and re-init the radio
        lora_ok = (SX127x_Init(LORA_FREQ_HZ, LORA_TX_DBM) == 0);
        lora_fail = 0;
        print("SX1278: TxDone timeout, re-init %s\r\n", lora_ok ? "ok" : "FAILED");
      }
      /* 3b. SPU status (phase, armed, pyros, battery) every 2 s while it is fresh */
      if (lora_ok && spu_count && (now - spu_ms) < 3000u && (now - last_spu_air_ms) >= 2000u) {
        last_spu_air_ms = now;
        SX127x_Send(spu_frame, (uint8_t)(sizeof(Athena_SpuStatus) + LINK_OVERHEAD));
        if (huart7.gState == HAL_UART_STATE_READY) {
          memcpy(bt_frame, spu_frame, sizeof(Athena_SpuStatus) + LINK_OVERHEAD);
          HAL_UART_Transmit_IT(&huart7, bt_frame, (uint16_t)(sizeof(Athena_SpuStatus) + LINK_OVERHEAD));
        }
      }
    }

    /* 4. uplink: anything the ground station sends is parsed with the same framing */
    if (lora_ok) {
      uint8_t rx[LINK_MAX_PAYLOAD + LINK_OVERHEAD];
      int n = SX127x_Receive(rx, sizeof rx, &last_rssi, &last_snr);
      for (int i = 0; i < n; i++) Link_FeedByte(&air_link, rx[i]);
    }

    /* 4b. anything the Bluetooth module said */
    if (bt_len == 0xFF) { print("bt: %s\r\n", bt_line); bt_len = 0; }

    /* 5. status over USB */
    if ((now - last_print_ms) >= STATUS_PRINT_MS) {
      last_print_ms = now;
      const Ublox_Hw *hw = Ublox_HwStatus();
      char logst[48]; Logger_StatusLine(logst, sizeof logst);
      static char line[320];
      int ln = snprintf(line, sizeof line, "gps fix=%u sv=%u lat=%ld lon=%ld hmsl=%ld m (n=%lu, %lu.%lu Hz) ant=%s pwr=%u noise=%u agc=%u jam=%u | mpu alt=%.1f vD=%.1f flags=0x%02X (n=%lu, %lu ms ago, bad=%lu) | spu %s ph=%u fl=0x%02X fired=0x%02X vbat=%u | air rx=%lu rssi=%d | %s | boot=%08lX/%08lX",
            gps.fix_type, gps.num_sv, (long)gps.lat_1e7, (long)gps.lon_1e7, (long)(gps.h_msl_mm / 1000), (unsigned long)gps_count,
            (unsigned long)(gps_rate_x10 / 10), (unsigned long)(gps_rate_x10 % 10),
            hw->valid ? Ublox_AntStatusStr(hw->ant_status) : "-", hw->ant_power, hw->noise_per_ms, hw->agc_cnt, hw->jam_ind,
            -mpu_state.pos_ned[2], mpu_state.vel_ned[2], mpu_state.flags, (unsigned long)state_count, (unsigned long)(HAL_GetTick() - mpu_state_ms), (unsigned long)mpu_link.rx_bad,
            (spu_count && (now - spu_ms) < 3000u) ? "ok" : "LOST", spu.phase, spu.flags, spu.pyro_fired, spu.vbat_mv,
            (unsigned long)air_count, last_rssi, logst, (unsigned long)dfu_boot_magic, (unsigned long)dfu_boot_bkp);
      if (ln > (int)sizeof line - 1) ln = sizeof line - 1;
      print("%s\r\n", line);
      static uint8_t text_frame[LINK_MAX_PAYLOAD + LINK_OVERHEAD];
      Logger_Write(text_frame, (uint32_t)Link_Encode(text_frame, LINK_PKT_TEXT, line, (uint8_t)(ln > LINK_MAX_PAYLOAD ? LINK_MAX_PAYLOAD : ln)));
      int mpu_alive = (now - mpu_state_ms) < 1000u && state_count;
      if (leds_off)                                   Set_LED_Color(LED_OFF);
      else if (HAL_GetTick() < LED_IDENTITY_MS)       { /* keep showing the identity colour */ }
      else if (!lora_ok)                              Set_LED_Color(LED_YELLOW);
      else if (mpu_alive && gps.fix_type >= 3)        Set_LED_Color(LED_GREEN);
      else if (mpu_alive)                             Set_LED_Color(LED_CYAN);
      else                                            Set_LED_Color(LED_BLUE);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 1;
  hfdcan2.Init.NominalTimeSeg2 = 1;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 0;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 32;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 7;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 24;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */
  return;   /* the microSD is removable: logger.c initialises SDMMC1 only when a card is present,
               so the generated HAL_SD_Init()/Error_Handler() below must never run */
  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 10;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_SLAVE;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 57600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart8.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RF_RESET_Pin|RF_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TPU_CAN_S_GPIO_Port, TPU_CAN_S_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TPU_R_Pin|GPS_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TPU_G_Pin|TPU_B_Pin|GPS_SAFEBOOT_Pin|GPS_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_SEL_GPIO_Port, GPS_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RF_RESET_Pin RF_CS_Pin */
  GPIO_InitStruct.Pin = RF_RESET_Pin|RF_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_DIO4_Pin RF_DIO5_Pin */
  GPIO_InitStruct.Pin = RF_DIO4_Pin|RF_DIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_DIO3_Pin RF_DIO2_Pin */
  GPIO_InitStruct.Pin = RF_DIO3_Pin|RF_DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_DIO1_Pin RF_DIO0_Pin */
  GPIO_InitStruct.Pin = RF_DIO1_Pin|RF_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_RESET_Pin */
  GPIO_InitStruct.Pin = BT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BT_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TPU_SELECT_Pin */
  GPIO_InitStruct.Pin = TPU_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TPU_SELECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TPU_CAN_S_Pin TPU_R_Pin GPS_RESET_Pin */
  GPIO_InitStruct.Pin = TPU_CAN_S_Pin|TPU_R_Pin|GPS_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TPU_G_Pin TPU_B_Pin GPS_SEL_Pin GPS_SAFEBOOT_Pin
                           GPS_CS_Pin */
  GPIO_InitStruct.Pin = TPU_G_Pin|TPU_B_Pin|GPS_SEL_Pin|GPS_SAFEBOOT_Pin
                          |GPS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CD_Pin GPS_LNA_EN_Pin */
  GPIO_InitStruct.Pin = SD_CD_Pin|GPS_LNA_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_EXTINT_Pin */
  GPIO_InitStruct.Pin = GPS_EXTINT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_EXTINT_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* --- crash recorder: fault handlers (it.c) branch here with the exception frame; Error_Handler() records too.
 * The record sits next to the DFU magic word in RAM that the startup code never touches, survives the
 * reset and is printed by the next boot. */
void Athena_FaultSave(uint32_t *sp)
{
  volatile uint32_t *r = (volatile uint32_t *)(DFU_MAGIC_ADDR + 0x10);
  r[1] = sp[6]; r[2] = sp[5]; r[3] = SCB->CFSR; r[4] = SCB->HFSR; r[5] = SCB->BFAR; r[0] = 0x46415554UL;   /* "FAUT" */
  __DSB();
  NVIC_SystemReset();
}
/* --- software entry into the ST ROM bootloader (USB DFU), two ways ---------------------------
 * 'B': leave a magic word in RAM and reset; main() checks it first thing and jumps (needs the
 *      SRAM word to survive the reset).
 * 'J': jump from here without a reset: tear USB, clocks, NVIC, caches and MPU down to their
 *      reset state and enter system memory with interrupts enabled (the ROM uses the USB IRQ). */
extern USBD_HandleTypeDef hUsbDeviceFS;
static volatile uint8_t dfu_request;
void Athena_DfuRequest(uint8_t c) { dfu_request = c; }

static void Athena_ResetToDfu(void)
{
  USBD_DeInit(&hUsbDeviceFS);                                   /* host sees a disconnect (>= 30 ms) before the reset */
  HAL_Delay(100);
  DFU_BKP_ENABLE();
  DFU_BKP_REG = DFU_MAGIC;                                      /* backup register: immune to whatever happens to SRAM */
  *(volatile uint32_t *)DFU_MAGIC_ADDR = DFU_MAGIC;             /* AXI SRAM, untouched by the startup code (all sections live in DTCM); D-cache is off */
  __DSB();
  NVIC_SystemReset();
}

static void Athena_JumpToBootloader(void)
{
  USBD_DeInit(&hUsbDeviceFS);                                   /* host sees a disconnect */
  HAL_Delay(100);
  __disable_irq();
  HAL_RCC_DeInit();
  HAL_DeInit();
  SysTick->CTRL = 0; SysTick->LOAD = 0; SysTick->VAL = 0;
  for (unsigned i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) { NVIC->ICER[i] = 0xFFFFFFFFu; NVIC->ICPR[i] = 0xFFFFFFFFu; }
#if defined(__CORTEX_M) && (__CORTEX_M == 7U)
  SCB_DisableICache();
  SCB_DisableDCache();
#endif
  HAL_MPU_Disable();
  SCB->VTOR = DFU_SYSMEM_ADDR;
  __set_MSP(*(volatile uint32_t *)DFU_SYSMEM_ADDR);
  __enable_irq();
  ((void (*)(void))(*(volatile uint32_t *)(DFU_SYSMEM_ADDR + 4)))();
  for (;;) {}
}

static void Athena_DfuPoll(void)                                /* main loop */
{
  uint8_t c = dfu_request;
  if (!c) return;
  dfu_request = 0;
  if (c == 'J') Athena_JumpToBootloader();
  else Athena_ResetToDfu();
}

static void on_mpu_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user)
{
  (void)user;
  if (type == LINK_PKT_STATE && len == sizeof(Athena_State)) {
    memcpy(&mpu_state, payload, sizeof mpu_state);
    mpu_state_ms = HAL_GetTick();
    state_count++;
    static uint8_t f[sizeof(Athena_State) + LINK_OVERHEAD];
    Logger_Write(f, (uint32_t)Link_Encode(f, LINK_PKT_STATE, payload, len));
  } else if (type == LINK_PKT_SPU && len == sizeof(Athena_SpuStatus)) {
    memcpy(&spu, payload, sizeof spu);
    spu_ms = HAL_GetTick(); spu_count++;
    size_t n = Link_Encode(spu_frame, LINK_PKT_SPU, payload, len);
    Logger_Write(spu_frame, (uint32_t)n);
    CDC_Transmit_FS(spu_frame, (uint16_t)n);           // USB dashboard (dropped if busy)
  }
}

/* commands pass through unchanged to the MPU, which hands them to the SPU (the SPU owns the safety checks) */
static void forward_cmd(const uint8_t *payload, uint8_t len, const char *src)
{
  cmd_count++;
  cmd_pending_len = (uint8_t)Link_Encode(cmd_pending, LINK_PKT_CMD, payload, len);
  print("cmd from %s: %u ch=%u val=%u -> MPU -> SPU\r\n", src, payload[0], payload[1], (unsigned)(payload[2] | payload[3] << 8));
}

static void on_bt_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user)
{
  (void)user;
  if (type == LINK_PKT_CMD && len == sizeof(Athena_Cmd)) forward_cmd(payload, len, "bt");   // phone/laptop over the DSPS bridge
  else if (type == LINK_PKT_TEXT) print("bt: %.*s\r\n", len, (const char *)payload);
}

static void on_bt_text(uint8_t b, void *user)        /* CodeLess AT replies, one line at a time (main loop context) */
{
  (void)user;
  if (bt_len == 0xFF) return;                          // previous line not printed yet: drop
  if (b == '\n' || bt_len >= sizeof(bt_line) - 1) { bt_line[bt_len] = 0; bt_len = 0xFF; }
  else if (b >= 32) bt_line[bt_len++] = (char)b;
}

static void on_usb_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user)
{
  (void)user;
  if (type == LINK_PKT_CMD && len == sizeof(Athena_Cmd)) forward_cmd(payload, len, "usb");
}

static void on_usb_text(uint8_t b, void *user)       /* single characters typed on the USB console */
{
  (void)user;
  if (b == 'B' || b == 'J') Athena_DfuRequest(b);
  else if (b == 'L') { leds_off = !leds_off; if (leds_off) Set_LED_Color(LED_OFF); print("leds %s\r\n", leds_off ? "off" : "on"); }
  else Logger_UsbRx(&b, 1);                            // 'D' dump flash log, 'E' restart it, 'S' sync SD, 'F' format SD
}

void Athena_UsbRx(const uint8_t *buf, uint32_t len)   /* USB CDC receive interrupt */
{
  for (uint32_t i = 0; i < len; i++) Link_RxPush(&usb_link, buf[i]);
}

static void on_air_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user)
{
  (void)user;
  air_count++;
  if (type == LINK_PKT_TEXT) print("air: %.*s\r\n", len, (const char *)payload);
  else if (type == LINK_PKT_CMD && len == sizeof(Athena_Cmd)) forward_cmd(payload, len, "air");   // ground station uplink
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART8) {
    Link_RxPush(&mpu_link, uart8_rx_byte);
    HAL_UART_Receive_IT(&huart8, &uart8_rx_byte, 1);
  } else if (huart->Instance == UART7) {                // decoded in the main loop (frames and text lines)
    bt_rx_bytes++;
    Link_RxPush(&bt_link, uart7_rx_byte);
    HAL_UART_Receive_IT(&huart7, &uart7_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART8) {              // an overrun aborts IT reception: re-arm it
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UART_Receive_IT(&huart8, &uart8_rx_byte, 1);
  } else if (huart->Instance == UART7) {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UART_Receive_IT(&huart7, &uart7_rx_byte, 1);
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  { volatile uint32_t *r = (volatile uint32_t *)(DFU_MAGIC_ADDR + 0x10);       /* who called us: printed after the reset */
    r[1] = (uint32_t)__builtin_return_address(0); r[2] = 0; r[3] = r[4] = r[5] = 0; r[0] = 0x4552524FUL;   /* "ERRO" */
    __DSB();
    NVIC_SystemReset(); }
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

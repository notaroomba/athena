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
#include "driver_bmp388_basic.h"
// #include "driver_bmp388_shot.h"
#include "icp201xx_interface.h"
#include "imu_interface.h"
#include "lis2mdl.h"
#include "athena_link.h"
#include "fusion.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define __FPU_USED 1U
#define FSYNC_FREQUENCY_HZ 30000  // FSYNC frequency at 30kHz
#define IMU_RATE_HZ    400        // fusion predict rate (= IMU ODR)
#define STATE_RATE_HZ  20         // Athena_State frames sent to the TPU
#define MAG_POLL_HZ    200
#define BARO_POLL_HZ   100
#define LED_IDENTITY_MS 3000      // show the MCU identity colour this long after reset
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi6;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint32_t dfu_boot_magic, dfu_boot_bkp;
static uint32_t fault_rec[6];                 // crash record from the previous run (magic, pc, lr, cfsr, hfsr, bfar)
Athena_LED_PinConfig led_pins = {
      .port_r = MPU_R_GPIO_Port,
      .pin_r = MPU_R_Pin,
      .port_g = MPU_G_GPIO_Port,
      .pin_g = MPU_G_Pin,
      .port_b = MPU_B_GPIO_Port,
      .pin_b = MPU_B_Pin
  };

volatile uint8_t bmp388_data_ready = 0;  // Flag set by interrupt

static Link           tpu_link;                 // UART4 <-> TPU
static uint8_t        uart4_rx_byte;
static uint8_t        state_frame[sizeof(Athena_State) + LINK_OVERHEAD];
static uint8_t        uart_frame[sizeof(Athena_State) + LINK_OVERHEAD];   // owned by the UART4 IT transfer
static Fusion         fusion;
static Athena_State   state;
static Athena_GpsFix  last_gps;
static uint32_t       gps_count;
static ICP201xx_t     icp_device;
static uint8_t        imu_mask, mag_ok, icp_ok;
/* in-flight reference, saved once a second above 20 m into RAM the startup code never touches, so a reset in
 * flight resumes with the pad's baro reference and GPS origin instead of re-zeroing at the current altitude */
typedef struct { uint32_t magic; float baro_alt0; double lat0, lon0; float h0; uint32_t sum; } FlightRec;
#define FLIGHT_REC ((volatile FlightRec *)(DFU_MAGIC_ADDR + 0x40))
#define FLIGHT_MAGIC 0x464C5431UL
static uint32_t flight_sum(const FlightRec *r) { const uint32_t *w = (const uint32_t *)r; uint32_t s = 0x5A5A; for (unsigned i = 1; i < sizeof(FlightRec) / 4 - 1; i++) s = s * 31u + w[i]; return s; }
static uint8_t flight_restored;
static Link           spu_link;                 // UART8 <-> SPU: state frames out, SPU status in
static Link           usb_link;                 // USB console: command frames from the dashboard, 'B'/'J'
static uint8_t        uart8_rx_byte;
static uint8_t        uart8_frame[sizeof(Athena_State) + LINK_OVERHEAD];      // owned by the UART8 IT transfer (state -> SPU)
static uint8_t        fwd_tpu_frame[sizeof(Athena_SpuStatus) + LINK_OVERHEAD], fwd_tpu_pending[sizeof(Athena_SpuStatus) + LINK_OVERHEAD];
static uint8_t        fwd_spu_frame[sizeof(Athena_Cmd) + LINK_OVERHEAD], fwd_spu_pending[sizeof(Athena_Cmd) + LINK_OVERHEAD];
static uint8_t        fwd_tpu_len, fwd_spu_len; // bytes waiting in the *_pending buffers for a free UART
static uint8_t        spu_usb_frame[sizeof(Athena_SpuStatus) + LINK_OVERHEAD];
static Athena_SpuStatus spu;
static uint32_t       spu_ms, spu_count, cmd_count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI6_Init(void);
static void MX_UART8_Init(void);
static void MX_UART4_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void Athena_DfuPoll(void);
void Athena_DfuRequest(uint8_t c);
static void on_tpu_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user);
static void on_spu_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user);
static void on_usb_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user);
static void on_usb_text(uint8_t b, void *user);
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
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif
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
  Athena_TimerConfig timer_cfg = {
    .htim = &htim2
  };
  Athena_Init(&led_pins, &timer_cfg);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI6_Init();
  MX_UART8_Init();
  MX_UART4_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Set_LED_Color(LED_GREEN);                   // identity colour: MPU = green (TPU red, SPU blue)
  HAL_TIM_Base_Start(&htim2);                 // 1 MHz free-running counter behind GetTimestamp()
  HAL_Delay(1000);                            // let USB CDC enumerate so the first prints are seen

  Link_Init(&tpu_link, on_tpu_packet, NULL);
  Link_Init(&spu_link, on_spu_packet, NULL);
  Link_Init(&usb_link, on_usb_packet, NULL);
  usb_link.on_text = on_usb_text;
  HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
  HAL_UART_Receive_IT(&huart8, &uart8_rx_byte, 1);
  Fusion_Init(&fusion, NULL);
  { FlightRec r; r.magic = FLIGHT_REC->magic; r.baro_alt0 = FLIGHT_REC->baro_alt0; r.lat0 = FLIGHT_REC->lat0; r.lon0 = FLIGHT_REC->lon0; r.h0 = FLIGHT_REC->h0; r.sum = FLIGHT_REC->sum;
    FLIGHT_REC->magic = 0;
    if (r.magic == FLIGHT_MAGIC && r.sum == flight_sum(&r)) {        /* we were flying when the reset hit */
      fusion.baro_alt0 = r.baro_alt0; fusion.have_baro0 = 1;
      fusion.lat0 = r.lat0; fusion.lon0 = r.lon0; fusion.h0 = r.h0; fusion.have_origin = 1;
      fusion.in_flight = 1; flight_restored = 1;
    } }

  print("\r\n=== Athena MPU ===\r\n");
  print("dfu: boot magic sram=0x%08lX bkp=0x%08lX\r\n", (unsigned long)dfu_boot_magic, (unsigned long)dfu_boot_bkp);
  if (fault_rec[0]) print("%s before this reset: pc=0x%08lX lr=0x%08lX cfsr=0x%08lX hfsr=0x%08lX bfar=0x%08lX\r\n",
                          fault_rec[0] == 0x46415554UL ? "FAULT" : "ERROR_HANDLER", (unsigned long)fault_rec[1], (unsigned long)fault_rec[2],
                          (unsigned long)fault_rec[3], (unsigned long)fault_rec[4], (unsigned long)fault_rec[5]);
  imu_mask = IMU_Init();
  print("IMU mask 0x%X (bit n = IMU n+1 alive)\r\n", imu_mask);
  if (flight_restored) print("fusion: in-flight reference restored after a reset (pad baro %.1f m, origin %.5f %.5f)\r\n", fusion.baro_alt0, fusion.lat0, fusion.lon0);
  mag_ok = (LIS2MDL_Init() == 0);
  print("LIS2MDL %s\r\n", mag_ok ? "ok" : "FAILED");
  ICP201xx_init_spi(&icp_device);
  icp_ok = (ICP201xx_begin(&icp_device) == 0) && (ICP201xx_start(&icp_device) == 0);
  print("ICP-20100 %s\r\n", icp_ok ? "ok" : "FAILED");
  // ponytail: BMP388 left out, its libdriver blocks for seconds; ICP is the primary baro.
  //           Add it as a second Fusion_Baro() call once a non-blocking read exists.
  uint32_t last_imu_us = GetTimestamp(), last_mag_us = 0, last_baro_us = 0;
  uint32_t last_state_us = 0, last_print_us = 0, last_hz_us = 0, loops = 0;
  uint16_t loop_hz = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = GetTimestamp();
    Athena_DfuPoll();

    /* 1. IMUs: vote the three units, convert to SI, run attitude + dead-reckoning predict */
    if ((now - last_imu_us) >= (1000000u / IMU_RATE_HZ)) {
      float dt = (float)(now - last_imu_us) * 1e-6f;
      last_imu_us = now;
      IMU_Data d;
      if (IMU_ReadFused(imu_mask, &d)) {
        float acc[3], gyr[3];
        for (int i = 0; i < 3; i++) { acc[i] = d.accel_g[i] * 9.80665f; gyr[i] = d.gyro_dps[i] * 0.017453292f; }
        Fusion_Imu(&fusion, acc, gyr, dt, now);
        loops++;
      }
    }

    /* 2. magnetometer: yaw reference for the attitude filter */
    if (mag_ok && (now - last_mag_us) >= (1000000u / MAG_POLL_HZ)) {
      last_mag_us = now;
      float m[3];
      if (LIS2MDL_Read(m) == 0) Fusion_Mag(&fusion, m, now);
    }

    /* 3. barometer: altitude measurement for the Down axis */
    if (icp_ok && (now - last_baro_us) >= (1000000u / BARO_POLL_HZ)) {
      last_baro_us = now;
      float p_kpa, t_c;
      if (ICP201xx_getData(&icp_device, &p_kpa, &t_c) == 0) Fusion_Baro(&fusion, p_kpa * 1000.f, now);
    }

    /* 4. GPS fixes and commands relayed by the TPU (on_tpu_packet), SPU status (on_spu_packet), USB console */
    Link_Process(&tpu_link);
    Link_Process(&spu_link);
    Link_Process(&usb_link);
    /* 4b. relay: SPU status -> TPU over UART4, commands -> SPU over UART8, as soon as the UART is free */
    if (fwd_tpu_len && huart4.gState == HAL_UART_STATE_READY) {
      memcpy(fwd_tpu_frame, fwd_tpu_pending, fwd_tpu_len);
      HAL_UART_Transmit_IT(&huart4, fwd_tpu_frame, fwd_tpu_len); fwd_tpu_len = 0;
    }
    if (fwd_spu_len && huart8.gState == HAL_UART_STATE_READY) {
      memcpy(fwd_spu_frame, fwd_spu_pending, fwd_spu_len);
      HAL_UART_Transmit_IT(&huart8, fwd_spu_frame, fwd_spu_len); fwd_spu_len = 0;
    }

    /* 5. publish the navigation state to the TPU */
    if ((now - last_state_us) >= (1000000u / STATE_RATE_HZ)) {
      last_state_us = now;
      if ((now - last_hz_us) >= 1000000u) { loop_hz = (uint16_t)loops; loops = 0; last_hz_us = now; }
      Fusion_GetState(&fusion, &state, imu_mask, loop_hz);
      size_t n = Link_Encode(state_frame, LINK_PKT_STATE, &state, sizeof state);
      CDC_Transmit_FS(state_frame, (uint16_t)n);               // same frame to the USB dashboard (dropped if busy)
      if (huart4.gState == HAL_UART_STATE_READY) {
        memcpy(uart_frame, state_frame, n);
        HAL_UART_Transmit_IT(&huart4, uart_frame, (uint16_t)n);
      }
      if (huart8.gState == HAL_UART_STATE_READY) {             // the SPU runs its recovery logic on the same frames
        memcpy(uart8_frame, state_frame, n);
        HAL_UART_Transmit_IT(&huart8, uart8_frame, (uint16_t)n);
      }
    }

    /* 6. human-readable status over USB (+ the in-flight reference snapshot, once a second) */
    if ((now - last_print_us) >= 200000u) {
      last_print_us = now;
      static uint8_t snap_div;
      if (fusion.in_flight && -state.pos_ned[2] > 20.f && ++snap_div >= 5) {
        snap_div = 0;
        FlightRec r = { FLIGHT_MAGIC, fusion.baro_alt0, fusion.lat0, fusion.lon0, fusion.h0, 0 }; r.sum = flight_sum(&r);
        FLIGHT_REC->baro_alt0 = r.baro_alt0; FLIGHT_REC->lat0 = r.lat0; FLIGHT_REC->lon0 = r.lon0; FLIGHT_REC->h0 = r.h0; FLIGHT_REC->sum = r.sum; FLIGHT_REC->magic = r.magic;
      }
      float roll, pitch, yaw;
      Fusion_QuatToEuler(state.q, &roll, &pitch, &yaw);
      print("alt %7.1f m  vD %6.1f m/s  baro %7.1f m | rpy %6.1f %6.1f %6.1f | imu 0x%X %u Hz | gps %s n=%lu rx_bad=%lu | %s | spu %s ph=%u fl=0x%02X vbat=%u | boot=%08lX/%08lX\r\n",
            -state.pos_ned[2], state.vel_ned[2], state.baro_alt,
            roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f,
            state.imu_mask, state.loop_hz,
            (state.flags & STATE_FLAG_GPS_FRESH) ? "fresh" : "DR", (unsigned long)gps_count, (unsigned long)tpu_link.rx_bad,
            (state.flags & STATE_FLAG_IN_FLIGHT) ? "FLIGHT" : "pad",
            (spu_count && HAL_GetTick() - spu_ms < 2000u) ? "ok" : "LOST", spu.phase, spu.flags, spu.vbat_mv, (unsigned long)dfu_boot_magic, (unsigned long)dfu_boot_bkp);
      if (HAL_GetTick() < LED_IDENTITY_MS)          { /* keep showing the identity colour */ }
      else if (state.flags & STATE_FLAG_IN_FLIGHT) Set_LED_Color(LED_MAGENTA);
      else if (!imu_mask)                          Set_LED_Color(LED_RED);
      else if (imu_mask == 0x7 && mag_ok && icp_ok && (state.flags & STATE_FLAG_GPS_FRESH)) Set_LED_Color(LED_GREEN);
      else                                         Set_LED_Color(LED_YELLOW);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 32;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
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
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
 uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();

// APB1 timer kernel clock (RM0433 8.5.8, TIMPRE = 0): equal to PCLK1 when the APB1
// prescaler is 1 (D2PPRE1 = 0xx), twice PCLK1 for any division (D2PPRE1 = 1xx, i.e. >= 4).
// This project runs APB1 at /2 (value 4), so the test must be ">= 4", not "> 4".
uint32_t ppre = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
if (ppre >= 4) tim_clk *= 2;

// 1 MHz tick (1 us resolution, wraps every 71 min) so GetTimestamp() returns microseconds
htim2.Init.Prescaler = (tim_clk / 1000000) - 1;
htim2.Init.Period    = 0xFFFFFFFF;

HAL_TIM_Base_Init(&htim2);   // <- THIS IS NOW SAFE
HAL_TIM_Base_Start(&htim2);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  HAL_GPIO_WritePin(TPU_SELECT_GPIO_Port, TPU_SELECT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, IMU1_CS_Pin|IMU2_CS_Pin|IMU3_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MPU_R_Pin|MPU_G_Pin|MPU_B_Pin|SPU_SELECT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MPU_CAN_S_Pin|ICP_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, BMP_CS_Pin|ICP_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : TPU_SELECT_Pin */
  GPIO_InitStruct.Pin = TPU_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TPU_SELECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MAG_CS_Pin MPU_R_Pin MPU_G_Pin MPU_B_Pin
                           SPU_SELECT_Pin */
  GPIO_InitStruct.Pin = MAG_CS_Pin|MPU_R_Pin|MPU_G_Pin|MPU_B_Pin
                          |SPU_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU1_INT_Pin IMU2_INT_Pin IMU3_INT_Pin */
  GPIO_InitStruct.Pin = IMU1_INT_Pin|IMU2_INT_Pin|IMU3_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU1_CS_Pin IMU2_CS_Pin IMU3_CS_Pin */
  GPIO_InitStruct.Pin = IMU1_CS_Pin|IMU2_CS_Pin|IMU3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MPU_CAN_S_Pin BMP_CS_Pin ICP_CS_Pin ICP_INT_Pin */
  GPIO_InitStruct.Pin = MPU_CAN_S_Pin|BMP_CS_Pin|ICP_CS_Pin|ICP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP_INT_Pin */
  GPIO_InitStruct.Pin = BMP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMP_INT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  
  /* Enable NVIC interrupt for EXTI15_10 (handles IMU1_INT on PE10, IMU2_INT on PE12, IMU3_INT on PE14) */
  /* IMU INT1 lines are not configured on the sensors and the filter polls the IMUs, so the
   * EXTI15_10 interrupt stays disabled: a floating INT line would otherwise fire it continuously. */
  
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

static void on_tpu_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user)
{
  (void)user;
  if (type == LINK_PKT_GPS && len == sizeof(Athena_GpsFix)) {
    memcpy(&last_gps, payload, sizeof last_gps);
    gps_count++;
    Fusion_Gps(&fusion, &last_gps, GetTimestamp());
  } else if (type == LINK_PKT_CMD && len == sizeof(Athena_Cmd)) {
    forward_cmd(payload, len, "tpu");                   // uplink / TPU console -> SPU
  }
}

/* commands pass through unchanged (re-framed) to the SPU, which owns the safety checks */
static void forward_cmd(const uint8_t *payload, uint8_t len, const char *src)
{
  cmd_count++;
  fwd_spu_len = (uint8_t)Link_Encode(fwd_spu_pending, LINK_PKT_CMD, payload, len);
  print("cmd from %s: %u -> SPU\r\n", src, payload[0]);
}

static void on_spu_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user)
{
  (void)user;
  if (type == LINK_PKT_SPU && len == sizeof(Athena_SpuStatus)) {
    memcpy(&spu, payload, sizeof spu);
    spu_ms = HAL_GetTick(); spu_count++;
    size_t n = Link_Encode(spu_usb_frame, LINK_PKT_SPU, payload, len);
    CDC_Transmit_FS(spu_usb_frame, (uint16_t)n);        // USB dashboard on the MPU port
    memcpy(fwd_tpu_pending, spu_usb_frame, n); fwd_tpu_len = (uint8_t)n;   // and on to the TPU: log + radio
  }
}

static void on_usb_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user)
{
  (void)user;
  if (type == LINK_PKT_CMD && len == sizeof(Athena_Cmd)) forward_cmd(payload, len, "usb");
}

static void on_usb_text(uint8_t b, void *user)
{
  (void)user;
  if (b == 'B' || b == 'J') Athena_DfuRequest(b);
}

void Athena_UsbRx(const uint8_t *buf, uint32_t len)      /* USB CDC receive interrupt */
{
  for (uint32_t i = 0; i < len; i++) Link_RxPush(&usb_link, buf[i]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4) {
    Link_RxPush(&tpu_link, uart4_rx_byte);
    HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
  } else if (huart->Instance == UART8) {
    Link_RxPush(&spu_link, uart8_rx_byte);
    HAL_UART_Receive_IT(&huart8, &uart8_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);   // an overrun aborts IT reception: re-arm it
  if (huart->Instance == UART4) HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
  else if (huart->Instance == UART8) HAL_UART_Receive_IT(&huart8, &uart8_rx_byte, 1);
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

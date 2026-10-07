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
#include <string.h>
#include "usbd_cdc_if.h"
#include "athena.h"
#include "athena_link.h"
#include "recovery.h"
#include "pd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_IDENTITY_MS 3000      // show the MCU identity colour this long after reset
#define SPU_STATUS_MS   500       // Athena_SpuStatus frames to the MPU (and USB)
#define STATUS_PRINT_MS 1000
 #define TPS25751_I2C_ADDR        0x20  // 7-bit I2C Address (HAL will shift to 0x40)
  #define LOAD_FULL_FLASH          1     // Set to 1 for full flash, 0 for low region only
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint32_t dfu_boot_magic;
static Link            mpu_link;                 // UART5 <-> MPU: state frames in, SPU status out, commands relayed from the TPU
static Link            usb_link;                 // USB console: command frames and single-character commands
static uint8_t         uart5_rx_byte;
static uint8_t         uart_frame[sizeof(Athena_SpuStatus) + LINK_OVERHEAD];   // owned by the UART5 IT transfer
static uint8_t         usb_frame[sizeof(Athena_SpuStatus) + LINK_OVERHEAD];
static Recovery        rec;                      // flight phase, pyro channels, servos
static PD              pd;                       // TPS25751 USB-PD controller + BQ25713 charger behind it
static Athena_State    mpu_state;
static uint32_t        mpu_state_ms, state_count, cmd_count, cmd_rejected;
  Athena_LED_PinConfig led_pins = {
      .port_r = SPU_R_GPIO_Port,
      .pin_r = SPU_R_Pin,
      .port_g = SPU_G_GPIO_Port,
      .pin_g = SPU_G_Pin,
      .port_b = SPU_B_GPIO_Port,
      .pin_b = SPU_B_Pin
  };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART5_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Athena_DfuPoll(void);
void Athena_DfuRequest(uint8_t c);
void Athena_UsbRx(const uint8_t *buf, uint32_t len);
static void on_mpu_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user);
static void on_usb_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user);
static void on_usb_text(uint8_t b, void *user);
static void handle_cmd(const Athena_Cmd *c, const char *src);

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
  if (dfu_boot_magic == DFU_MAGIC) {                            /* 'B' on the USB console asked for DFU */
    *(volatile uint32_t *)DFU_MAGIC_ADDR = 0;
    SysTick->CTRL = 0;
    SCB->VTOR = DFU_SYSMEM_ADDR;                                /* ROM vector table; interrupts stay enabled as after a real reset */
    __set_MSP(*(volatile uint32_t *)DFU_SYSMEM_ADDR);
    ((void (*)(void))(*(volatile uint32_t *)(DFU_SYSMEM_ADDR + 4)))();   /* never returns */
  }

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  Athena_Init(&led_pins, NULL);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_FDCAN2_Init();
  MX_USART1_UART_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  Set_LED_Color(LED_BLUE);                             // identity colour: SPU = blue (MPU green, TPU red)
  Recovery_Init(&rec, NULL);                           // pyro outputs low, servos without pulse, main chute at 150 m
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);            // 50 Hz servo frames, pulse set in Recovery_HwServo()
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  Link_Init(&mpu_link, on_mpu_packet, NULL);
  Link_Init(&usb_link, on_usb_packet, NULL);
  usb_link.on_text = on_usb_text;
  HAL_UART_Receive_IT(&huart5, &uart5_rx_byte, 1);
  HAL_Delay(1000);                                     // USB CDC enumeration

  print("\r\n=== Athena SPU ===\r\n");
  print("dfu: magic word at boot was 0x%08lX\r\n", (unsigned long)dfu_boot_magic);
  PD_Init(&pd, &hi2c1);                                // TPS25751: patch it if it waits in PTCH mode, then it owns the charger
  print("init: done, main chute at %u m, disarmed\r\n", (unsigned)rec.p.main_alt_m);

  uint32_t last_status_ms = 0, last_print_ms = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    Athena_DfuPoll();

    /* 1. navigation state from the MPU (-> Recovery_OnState) and commands relayed from the TPU */
    Link_Process(&mpu_link);
    /* 2. USB console: command frames from the dashboard, single characters from a terminal */
    Link_Process(&usb_link);
    /* 3. pyro pulse timing, auto-disarm after landing */
    Recovery_Task(&rec, now);
    /* 4. USB-PD controller and charger, 1 Hz */
    PD_Task(&pd, now);

    /* 5. status frame to the MPU (forwarded to the TPU: log + telemetry) and to USB */
    if ((now - last_status_ms) >= SPU_STATUS_MS) {
      last_status_ms = now;
      Athena_SpuStatus st; memset(&st, 0, sizeof st);
      st.t_ms = now;
      Recovery_Fill(&rec, &st, now);
      PD_Fill(&pd, &st);
      if (HAL_GPIO_ReadPin(CHRG_OK_GPIO_Port, CHRG_OK_Pin) == GPIO_PIN_SET)       st.flags |= SPU_FLAG_CHRG_OK;
      if (HAL_GPIO_ReadPin(SPU_PROCHOT_GPIO_Port, SPU_PROCHOT_Pin) == GPIO_PIN_RESET) st.flags |= SPU_FLAG_PROCHOT;   // active low
      if (HAL_GPIO_ReadPin(CMPOUT_GPIO_Port, CMPOUT_Pin) == GPIO_PIN_SET)         st.flags |= SPU_FLAG_CMPOUT;
      size_t n = Link_Encode(usb_frame, LINK_PKT_SPU, &st, sizeof st);
      CDC_Transmit_FS(usb_frame, (uint16_t)n);                 // dropped if the endpoint is busy
      if (huart5.gState == HAL_UART_STATE_READY) {
        memcpy(uart_frame, usb_frame, n);
        HAL_UART_Transmit_IT(&huart5, uart_frame, (uint16_t)n);
      }
    }

    /* 6. human-readable status + LED */
    if ((now - last_print_ms) >= STATUS_PRINT_MS) {
      last_print_ms = now;
      int mpu_alive = state_count && (now - mpu_state_ms) < 1000u;
      print("spu %s%s | mpu %s alt=%.1f vz=%.1f (n=%lu, bad=%lu) | pyro fired=0x%02X on=0x%02X main=%um apogee=%.0fm | pd %s plug=%u vbat=%umV vbus=%umV ibat=%dmA iin=%umA chg=0x%04X | chrg_ok=%u prochot=%u cmpout=%u | cmd ok=%lu rej=%lu\r\n",
            Recovery_PhaseName(rec.phase), rec.armed ? " ARMED" : "",
            mpu_alive ? "ok" : "LOST", -mpu_state.pos_ned[2], -mpu_state.vel_ned[2], (unsigned long)state_count, (unsigned long)mpu_link.rx_bad,
            rec.fired, rec.on, (unsigned)rec.p.main_alt_m, rec.apogee_m,
            PD_ModeName(pd.mode), pd.status[0] & 1u, pd.vbat_mv, pd.vbus_mv, pd.ibat_ma, pd.iin_ma, pd.chg_status,
            HAL_GPIO_ReadPin(CHRG_OK_GPIO_Port, CHRG_OK_Pin), !HAL_GPIO_ReadPin(SPU_PROCHOT_GPIO_Port, SPU_PROCHOT_Pin), HAL_GPIO_ReadPin(CMPOUT_GPIO_Port, CMPOUT_Pin),
            (unsigned long)cmd_count, (unsigned long)cmd_rejected);
      if (HAL_GetTick() < LED_IDENTITY_MS)          { /* keep showing the identity colour */ }
      else if (rec.armed)                           Set_LED_Color((now / 250) & 1 ? LED_RED : LED_OFF);   // armed: blinking red
      else if (rec.phase == SPU_PHASE_LANDED)       Set_LED_Color(LED_CYAN);
      else if (rec.phase != SPU_PHASE_PAD)          Set_LED_Color(LED_MAGENTA);
      else if (!mpu_alive)                          Set_LED_Color(LED_YELLOW);
      else                                          Set_LED_Color(LED_GREEN);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV9;
  RCC_OscInitStruct.PLL.PLLN = 108;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
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
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60715075;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 143;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 143;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPU_SELECT_Pin|PYRO_3_Pin|PYRO_2_Pin|PYRO_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SERVO1_EN_Pin|SERVO2_EN_Pin|SERVO3_EN_Pin|SERVO4_EN_Pin
                          |SERVO5_EN_Pin|EN_OTG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPU_B_Pin|SPU_G_Pin|SPU_R_Pin|RESET_MPU_Pin|SPU_CAN_S_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SERVO6_EN_Pin|PYRO_6_Pin|PYRO_5_Pin|PYRO_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPU_SELECT_Pin PYRO_3_Pin PYRO_2_Pin PYRO_1_Pin */
  GPIO_InitStruct.Pin = SPU_SELECT_Pin|PYRO_3_Pin|PYRO_2_Pin|PYRO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD_IRQ_Pin (TPS25751 I2Cc_IRQ input: needs a pull-up, must never be driven low) */
  GPIO_InitStruct.Pin = PD_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PD_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SERVO1_EN_Pin SERVO2_EN_Pin SERVO3_EN_Pin SPU_B_Pin
                           SPU_G_Pin SPU_R_Pin SERVO4_EN_Pin SERVO5_EN_Pin
                           EN_OTG_Pin SPU_CAN_S_Pin RESET_MPU_Pin */
  GPIO_InitStruct.Pin = SERVO1_EN_Pin|SERVO2_EN_Pin|SERVO3_EN_Pin|SPU_B_Pin
                          |SPU_G_Pin|SPU_R_Pin|SERVO4_EN_Pin|SERVO5_EN_Pin
                          |EN_OTG_Pin|SPU_CAN_S_Pin|RESET_MPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CHRG_OK_Pin (BQ25713 open-drain output, pulled up on the board) */
  GPIO_InitStruct.Pin = CHRG_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHRG_OK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SERVO6_EN_Pin PYRO_6_Pin PYRO_5_Pin PYRO_4_Pin */
  GPIO_InitStruct.Pin = SERVO6_EN_Pin|PYRO_6_Pin|PYRO_5_Pin|PYRO_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPU_PROCHOT_Pin CMPOUT_Pin (BQ25713 open-drain outputs, pulled up on the board) */
  GPIO_InitStruct.Pin = SPU_PROCHOT_Pin|CMPOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPU_PD_IRQ_Pin */
  GPIO_InitStruct.Pin = SPU_PD_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPU_PD_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* --- recovery hardware hooks -------------------------------------------------------------------
 * Pyro channels: PYRO_n -> 1 k -> gate of a 2N7002 whose drain sits on the fused pyro bus (ARM terminal
 * in series with the battery) and whose source feeds the igniter terminal, so HIGH = conducting.
 * Servo power: SERVOn_EN drives the gate of an IRLML6402 P-FET between +5V and the servo header. With a
 * 3.3 V gate against a 5 V source (Vgs = -1.7 V) that FET conducts whatever the pin does; LOW gives full
 * enhancement, so the pins idle low and servo power is simply always on. ponytail: a board revision needs a
 * level shifter or an open-drain pin with a pull-up to +5V before "servo power off" can exist. */
static GPIO_TypeDef *const pyro_port[RECOVERY_PYRO_CH] = { PYRO_1_GPIO_Port, PYRO_2_GPIO_Port, PYRO_3_GPIO_Port, PYRO_4_GPIO_Port, PYRO_5_GPIO_Port, PYRO_6_GPIO_Port };
static const uint16_t   pyro_pin[RECOVERY_PYRO_CH]   = { PYRO_1_Pin, PYRO_2_Pin, PYRO_3_Pin, PYRO_4_Pin, PYRO_5_Pin, PYRO_6_Pin };

void Recovery_HwPyro(uint8_t ch, int on)
{
  if (ch < RECOVERY_PYRO_CH) HAL_GPIO_WritePin(pyro_port[ch], pyro_pin[ch], on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Recovery_HwServo(uint8_t ch, uint16_t us)       /* TIM1 CH1-4 = servo 1-4, TIM2 CH1-2 = servo 5-6; 1 us per tick, 20 ms frame */
{
  if (ch < 4)      __HAL_TIM_SET_COMPARE(&htim1, (uint32_t)ch * 4u, us);
  else if (ch < 6) __HAL_TIM_SET_COMPARE(&htim2, (uint32_t)(ch - 4) * 4u, us);
}

/* --- link handlers ------------------------------------------------------------------------------ */
static const char *const cmd_names[] = { "?", "ping", "arm", "disarm", "fire", "servo", "reset-mpu", "main-alt" };

static void handle_cmd(const Athena_Cmd *c, const char *src)
{
  const char *name = c->cmd < 8 ? cmd_names[c->cmd] : "?";
  if (c->cmd == CMD_RESET_MPU) {                       /* PB9 -> diode -> MPU NRST: a 20 ms low pulse */
    HAL_GPIO_WritePin(RESET_MPU_GPIO_Port, RESET_MPU_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(RESET_MPU_GPIO_Port, RESET_MPU_Pin, GPIO_PIN_SET);
    cmd_count++;
    print("cmd %s: %s -> MPU reset pulsed\r\n", src, name);
    return;
  }
  int rc = Recovery_Command(&rec, c, HAL_GetTick());
  if (rc == 0) cmd_count++; else cmd_rejected++;
  print("cmd %s: %s ch=%u val=%u -> %s%s\r\n", src, name, c->arg, c->value, rc == 0 ? "ok" : "REJECTED",
        rc == 0 && c->cmd == CMD_ARM ? " (pyros live when the ARM terminal is closed)" : "");
}

static void on_mpu_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user)
{
  (void)user;
  if (type == LINK_PKT_STATE && len == sizeof(Athena_State)) {
    memcpy(&mpu_state, payload, sizeof mpu_state);
    mpu_state_ms = HAL_GetTick(); state_count++;
    Recovery_OnState(&rec, &mpu_state, mpu_state_ms);
  } else if (type == LINK_PKT_CMD && len == sizeof(Athena_Cmd)) {
    Athena_Cmd c; memcpy(&c, payload, sizeof c);
    handle_cmd(&c, "link");
  }
}

static void on_usb_packet(uint8_t type, const uint8_t *payload, uint8_t len, void *user)
{
  (void)user;
  if (type == LINK_PKT_CMD && len == sizeof(Athena_Cmd)) {
    Athena_Cmd c; memcpy(&c, payload, sizeof c);
    handle_cmd(&c, "usb");
  }
}

/* single characters typed on the USB console (bench use): A arm, d disarm, 1-6 fire, s servo sweep, r reset MPU, B/J DFU */
static void on_usb_text(uint8_t b, void *user)
{
  (void)user;
  Athena_Cmd c; memset(&c, 0, sizeof c);
  if (b == 'B' || b == 'J') { Athena_DfuRequest(b); return; }
  if (b == 'A')      { c.cmd = CMD_ARM; c.key = CMD_KEY; }
  else if (b == 'd') { c.cmd = CMD_DISARM; }
  else if (b >= '1' && b <= '6') { c.cmd = CMD_FIRE; c.arg = (uint8_t)(b - '0'); c.key = CMD_KEY; }
  else if (b == 'r') { c.cmd = CMD_RESET_MPU; }
  else if (b == 's') { c.cmd = CMD_SERVO; c.arg = 1; c.value = rec.servo_us[0] == 1000 ? 2000 : 1000; }   // toggles servo 1 between its ends
  else return;
  handle_cmd(&c, "console");
}

void Athena_UsbRx(const uint8_t *buf, uint32_t len)       /* USB CDC receive interrupt */
{
  for (uint32_t i = 0; i < len; i++) Link_RxPush(&usb_link, buf[i]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5) {
    Link_RxPush(&mpu_link, uart5_rx_byte);
    HAL_UART_Receive_IT(&huart5, &uart5_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5) {                        // an overrun aborts IT reception: re-arm it
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UART_Receive_IT(&huart5, &uart5_rx_byte, 1);
  }
}

/* --- software entry into the ST ROM bootloader (USB DFU), two ways ---------------------------
 * 'B': disconnect USB so the host notices (>= 30 ms), leave a magic word in RAM and reset; main() checks
 *      it before anything else and jumps into system memory from a reset-clean chip.
 * 'J': jump from here without a reset: tear USB, clocks, NVIC and caches down to their reset state and
 *      enter system memory with interrupts enabled (the ROM uses the USB IRQ). */
extern USBD_HandleTypeDef hUsbDeviceFS;
static volatile uint8_t dfu_request;
void Athena_DfuRequest(uint8_t c) { dfu_request = c; }

static void Athena_ResetToDfu(void)
{
  USBD_DeInit(&hUsbDeviceFS);                                   /* host sees a disconnect */
  HAL_Delay(100);
  *(volatile uint32_t *)DFU_MAGIC_ADDR = DFU_MAGIC;
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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

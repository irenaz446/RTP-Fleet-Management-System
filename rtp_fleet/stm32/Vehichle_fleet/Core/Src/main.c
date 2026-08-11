/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Vehicle Fleet Management System V2 — STM32 I2C Slave
  *
  * Responsibilities of this file (main.c):
  *   - HAL peripheral initialisation
  *   - TIM2 state machine (TRIP_START → ACTIVE → END → WAIT)
  *   - Double-buffer pattern for safe I2C frame updates
  *   - IRQ callbacks (I2C TX complete, TIM2 tick)
  *
  * Sensor simulation and frame building → fleet_data.c / fleet_data.h
  *
  * Peripherals:
  *   I2C2   slave, address 0x08, sends 84-byte frame when BBG reads
  *   TIM2   1 Hz interrupt  (Prescaler=15999, Period=999, HSI 16MHz)
  *   USART3 115200 baud debug output
  *
  * Wiring to BBG:
  *   PB10 (SCL) ──[4.7kΩ to 3.3V]── P9_19
  *   PB11 (SDA) ──[4.7kΩ to 3.3V]── P9_20
  *   GND        ──────────────────── P9_1
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "fleet_data.h"   /* telemetry_frame_t, fleet_data_init/update/get_frame */
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
typedef enum {
    STATE_TRIP_START,
    STATE_TRIP_ACTIVE,
    STATE_TRIP_END,
    STATE_TRIP_WAIT
} app_state_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define I2C_SLAVE_ADDR      0x08
#define TRIP_DURATION_SEC   120   /* 2 minutes per trip      */
#define WAIT_BETWEEN_SEC    10    /* 10 seconds between trips*/
/* USER CODE END PD */

/* CubeMX handles */
I2C_HandleTypeDef  hi2c2;
TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/**
 * @brief Active frame buffer — transmitted to BBG via I2C.
 *        Only written inside HAL_I2C_SlaveTxCpltCallback (bus idle).
 */
static telemetry_frame_t g_frame;

/**
 * @brief Staging buffer — written by the main loop via fleet_get_frame().
 *        Promoted into g_frame in the TX callback when the bus is idle.
 *        Prevents race condition: memset inside fleet_get_frame() cannot
 *        zero g_frame while I2C DMA is simultaneously reading it.
 */
static telemetry_frame_t g_next_frame;

/** @brief Set by main loop when g_next_frame is ready to promote. */
static volatile uint8_t  g_frame_ready = 0;

/** @brief 1-second tick set by TIM2 IRQ — acts on in main loop only. */
static volatile uint8_t  g_tick        = 0;

/** @brief TX complete flag — print outside IRQ to avoid printf in ISR. */
static volatile uint8_t  g_tx_done     = 0;

static app_state_t g_state     = STATE_TRIP_START;
static uint32_t    g_sec_count = 0;
static uint32_t    g_total_tx  = 0;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN 0 */

/** @brief Arm the I2C slave — call from main loop only, never from IRQ. */
static void i2c_arm(void)
{
    HAL_I2C_Slave_Transmit_IT(&hi2c2,
                               (uint8_t *)&g_frame,
                               FRAME_SIZE);
}

/**
 * @brief I2C TX complete — swap staging buffer into active frame.
 *
 * This callback fires after the BBG has read all FRAME_SIZE bytes.
 * The I2C bus is idle at this point — safe to update g_frame.
 * Re-arms immediately so the slave is ready for the next BBG read.
 */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C2) return;

    /* Bus idle — safe to promote staging buffer into active buffer */
    if (g_frame_ready) {
        g_frame       = g_next_frame;   /* atomic struct copy */
        g_frame_ready = 0;
    }

    g_tx_done = 1;
    HAL_I2C_Slave_Transmit_IT(&hi2c2, (uint8_t *)&g_frame, FRAME_SIZE);
}

/** @brief I2C error — re-arm so slave stays responsive. */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C2) return;
    HAL_I2C_Slave_Transmit_IT(&hi2c2, (uint8_t *)&g_frame, FRAME_SIZE);
}

/** @brief TIM2 fires every 1 second — set flag only, no logic here. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;
    g_tick = 1;
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_USART3_UART_Init();
    MX_I2C2_Init();

    /* USER CODE BEGIN 2 */

    /* Verify frame size and field offsets at startup */
    printf("=== Vehicle Fleet STM32 V2 starting ===\r\n");
    printf("sizeof(telemetry_frame_t) = %d (expected %d)\r\n",
           (int)sizeof(telemetry_frame_t), FRAME_SIZE);
    printf("offsetof(msg_type)        = %d (expected 72)\r\n",
           (int)__builtin_offsetof(telemetry_frame_t, msg_type));
    printf("offsetof(altitude)        = %d (expected 36)\r\n",
           (int)__builtin_offsetof(telemetry_frame_t, altitude));

    if ((int)sizeof(telemetry_frame_t) != FRAME_SIZE) {
        printf("[ERROR] Frame size mismatch! Halting.\r\n");
        Error_Handler();
    }

    /* Initialise sensor simulation */
    fleet_data_init(HAL_GetTick() / 1000UL);

    /* Prepare initial frame and arm I2C slave */
    fleet_get_frame(&g_frame, MSG_TRIP_DATA);
    i2c_arm();

    /* Start 1-second timer */
    HAL_TIM_Base_Start_IT(&htim2);

    printf("Waiting for BBG master...\r\n\n");

    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */

        /* ── Print TX completion outside IRQ ─────────────────────── */
        if (g_tx_done) {
            g_tx_done = 0;
            g_total_tx++;

            static uint32_t stat_count = 0;
            if (++stat_count % 60 == 0) {
                printf("[STATS] tx=%lu type=%c lat=%.4f lon=%.4f "
                       "alt=%.1fm spd=%.1fkm/h\r\n",
                       (unsigned long)g_total_tx,
                       g_frame.msg_type,
                       g_frame.latitude,
                       g_frame.longitude,
                       g_frame.altitude,
                       g_frame.gps_speed_kmh);
            }
        }

        /* ── 1-second state machine ───────────────────────────────── */
        if (!g_tick) continue;
        g_tick = 0;

        fleet_data_update();   /* advance GPS position, altitude drift */

        switch (g_state) {

            case STATE_TRIP_START:
                fleet_get_frame(&g_next_frame, MSG_TRIP_START);
                g_frame_ready = 1;
                g_sec_count   = 0;
                g_state       = STATE_TRIP_ACTIVE;
                printf("[TRIP] Started\r\n");
                break;

            case STATE_TRIP_ACTIVE:
                fleet_get_frame(&g_next_frame, MSG_TRIP_DATA);
                g_frame_ready = 1;
                g_sec_count++;
                if (g_sec_count >= TRIP_DURATION_SEC)
                    g_state = STATE_TRIP_END;
                break;

            case STATE_TRIP_END:
                fleet_get_frame(&g_next_frame, MSG_TRIP_END);
                g_frame_ready = 1;
                g_sec_count   = 0;
                g_state       = STATE_TRIP_WAIT;
                printf("[TRIP] Ended after %d sec\r\n", TRIP_DURATION_SEC);
                break;

            case STATE_TRIP_WAIT:
                g_sec_count++;
                if (g_sec_count >= WAIT_BETWEEN_SEC) {
                    g_sec_count = 0;
                    g_state     = STATE_TRIP_START;
                    printf("[TRIP] Starting next trip...\r\n");
                }
                break;
        }

        /* USER CODE END 3 */
    }
}

/* ── CubeMX generated functions ──────────────────────────────────────── */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

static void MX_I2C2_Init(void)
{
    hi2c2.Instance              = I2C2;
    hi2c2.Init.Timing           = 0x00303D5B;
    hi2c2.Init.OwnAddress1      = I2C_SLAVE_ADDR << 1;
    hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2      = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
        Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 15999;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 999;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
        Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = 115200;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert: %s line %lu\r\n", (char *)file, (unsigned long)line);
}
#endif

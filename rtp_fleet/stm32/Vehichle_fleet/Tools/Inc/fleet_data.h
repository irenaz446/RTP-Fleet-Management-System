/**
 * @file fleet_data.h
 * @brief Vehicle Fleet Management System V3 — real-sensor layer
 *        Replaces the simulated fleet_data.c/h from V2.
 *
 *  Sensors added:
 *    MPU-6050  IMU  — I2C1 master  (PB6 SCL, PB7 SDA)
 *    NEO-6M/8M GPS  — USART1 RX   (PA10)
 *
 *  Everything else (frame layout, BBG, server) is IDENTICAL to V2.
 *  Drop these two files into Tools/Inc/ and Tools/Src/ and add
 *  I2C1 + USART1 inits to main.c (see main.c).
 *
 *  Target: STM32F756ZG  (Nucleo-144)
 *  HAL:    stm32f7xx_hal
 */

#ifndef FLEET_DATA_H
#define FLEET_DATA_H

#include "main.h"       /* pulls in stm32f7xx_hal.h, Error_Handler, etc. */
#include <stdint.h>
#include <string.h>

/* ─────────────────────────────────────────────────────────────────────
 *  MPU-6050  I2C definitions
 * ───────────────────────────────────────────────────────────────────── */
#define MPU6050_ADDR        (0x68U)           /* AD0 pin tied to GND      */
#define MPU6050_ADDR_W      (MPU6050_ADDR << 1)  /* HAL uses 8-bit address */

/* Registers */
#define MPU6050_PWR_MGMT_1  0x6BU   /* write 0x00 to wake                */
#define MPU6050_ACCEL_XOUT  0x3BU   /* first of 14 bytes: ax ay az T gx gy gz */

/* Default full-scale sensitivities */
#define MPU6050_ACCEL_SCALE 16384.0f  /* LSB/g   at ±2 g   */
#define MPU6050_GYRO_SCALE  131.0f    /* LSB/°/s at ±250°/s */

/* ─────────────────────────────────────────────────────────────────────
 *  GPS UART ring-buffer
 * ───────────────────────────────────────────────────────────────────── */
#define GPS_BUF_SIZE  256U                    /* must be power-of-two     */
#define GPS_BUF_MASK  (GPS_BUF_SIZE - 1U)

/* ─────────────────────────────────────────────────────────────────────
 *  Message type bytes  (shared with common.h on BBG/server)
 * ───────────────────────────────────────────────────────────────────── */
#define MSG_TRIP_START  'S'
#define MSG_TRIP_DATA   'D'
#define MSG_TRIP_END    'E'

/* ─────────────────────────────────────────────────────────────────────
 *  Frame constants
 * ───────────────────────────────────────────────────────────────────── */
#define FRAME_SIZE       84U
#define FRAME_SYNC_BYTE  0xABU

/* ─────────────────────────────────────────────────────────────────────
 *  Telemetry frame — 84 bytes, natural compiler layout.
 *  Layout IDENTICAL to V2 common.h — BBG and server unchanged.
 *
 *  [0-7]   car_id[8]
 *  [8-27]  trip_id[20]
 *  [28-31] latitude        float, decimal degrees
 *  [32-35] longitude       float, decimal degrees
 *  [36-39] altitude        float, metres ASL
 *  [40-43] gps_speed_kmh   float, km/h
 *  [44-47] accel_x         float, m/s²
 *  [48-51] accel_y         float, m/s²
 *  [52-55] accel_z         float, m/s²
 *  [56-59] gyro_x          float, °/s
 *  [60-63] gyro_y          float, °/s
 *  [64-67] gyro_z          float, °/s
 *  [68-71] timestamp_sec   uint32
 *  [72]    msg_type        'S','D','E'
 *  [73]    gps_fix         0=none, 1=GPS, 2=DGPS
 *  [74]    satellites      count
 *  [75-83] reserved[9]     [0]=0xAB sync marker
 *  Total = 84 bytes
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
    char     car_id[8];
    char     trip_id[20];
    float    latitude;
    float    longitude;
    float    altitude;
    float    gps_speed_kmh;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
    uint32_t timestamp_sec;
    uint8_t  msg_type;
    uint8_t  gps_fix;
    uint8_t  satellites;
    uint8_t  reserved[9];   /* [0] = FRAME_SYNC_BYTE = 0xAB */
} telemetry_frame_t;        /* total = 84 bytes              */

/* Compile-time size guard */
_Static_assert(sizeof(telemetry_frame_t) == FRAME_SIZE,
               "telemetry_frame_t must be exactly 84 bytes");

/* ─────────────────────────────────────────────────────────────────────
 *  GPS ring-buffer — written in UART ISR, read in main loop.
 *  Declared here so HAL_UART_RxCpltCallback (in fleet_data.c) can
 *  see them without extern declarations in main.c.
 * ───────────────────────────────────────────────────────────────────── */
extern volatile uint8_t  g_gps_rx_buf[GPS_BUF_SIZE];
extern volatile uint16_t g_gps_head;   /* written by ISR only  */
extern volatile uint16_t g_gps_tail;   /* written by main loop */
extern volatile uint8_t  g_gps_byte;   /* single-byte receive  */

/* ─────────────────────────────────────────────────────────────────────
 *  Public API  (mirrors V2's fleet_data_init / update / get_frame)
 * ───────────────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise real sensors.
 *         - Wakes MPU-6050 (clears PWR_MGMT_1 via I2C1).
 *         - Arms GPS UART interrupt receive (USART1).
 *         Call once after MX_I2C1_Init() and MX_USART1_UART_Init().
 *
 * @param  hi2c_imu   Pointer to I2C1 handle  (MPU-6050 master)
 * @param  huart_gps  Pointer to USART1 handle (GPS 9600 baud RX)
 */
void fleet_sensor_init(I2C_HandleTypeDef  *hi2c_imu,
                       UART_HandleTypeDef *huart_gps);

/**
 * @brief  Read one sample from MPU-6050 and drain the GPS ring-buffer.
 *         Replaces V2's fleet_data_update().
 *         Call from the 1 Hz TIM2 tick flag in the main loop.
 */
void fleet_data_update(void);

/**
 * @brief  Fill *frame with the latest sensor readings.
 *         Replaces V2's fleet_get_frame().
 *
 * @param  frame  Destination buffer (sizeof == FRAME_SIZE).
 * @param  type   MSG_TRIP_START / MSG_TRIP_DATA / MSG_TRIP_END
 */
void fleet_get_frame(telemetry_frame_t *frame, char type);

#endif /* FLEET_DATA_H */

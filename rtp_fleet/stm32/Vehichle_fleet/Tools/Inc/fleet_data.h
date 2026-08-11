/**
 * @file fleet_data.h
 * @brief Sensor simulation and telemetry frame API — V2 single car.
 *
 * Provides:
 *   - telemetry_frame_t struct (84 bytes, matches BBG/server common.h)
 *   - fleet_data_init()   — call once at startup
 *   - fleet_data_update() — call every 1 second (from TIM2 tick)
 *   - fleet_get_frame()   — fill a frame for the current trip state
 */

#ifndef FLEET_DATA_H
#define FLEET_DATA_H

#include <stdint.h>

/* ── Message type bytes ──────────────────────────────────────────────── */
#define MSG_TRIP_START  'S'
#define MSG_TRIP_DATA   'D'
#define MSG_TRIP_END    'E'

/* ── Frame size ──────────────────────────────────────────────────────── */
#define FRAME_SIZE      84

/* ── Sync marker (reserved[0]) ───────────────────────────────────────── */
#define FRAME_SYNC_BYTE 0xAB

/**
 * @brief Telemetry frame — 84 bytes, natural compiler layout.
 *
 * Layout (verified with offsetof on STM32 arm-none-eabi-gcc):
 *   [0-7]   car_id[8]
 *   [8-27]  trip_id[20]
 *   [28-31] latitude        float, degrees
 *   [32-35] longitude       float, degrees
 *   [36-39] altitude        float, metres above sea level
 *   [40-43] gps_speed_kmh   float, km/h
 *   [44-47] accel_x         float, m/s²
 *   [48-51] accel_y         float, m/s²
 *   [52-55] accel_z         float, m/s²
 *   [56-59] gyro_x          float, °/s
 *   [60-63] gyro_y          float, °/s
 *   [64-67] gyro_z          float, °/s
 *   [68-71] timestamp_sec   uint32
 *   [72]    msg_type        'S','D','E'
 *   [73]    gps_fix         uint8
 *   [74]    satellites      uint8
 *   [75-83] reserved[9]    [0]=0xAB sync marker
 *   Total  = 84 bytes
 */
typedef struct {
    char     car_id[8];
    char     trip_id[20];
    float    latitude;
    float    longitude;
    float    altitude;        /* metres above sea level — V2 addition */
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
    uint8_t  reserved[9];    /* [0] = FRAME_SYNC_BYTE = 0xAB */
} telemetry_frame_t;

/* ── Public API ──────────────────────────────────────────────────────── */

/**
 * @brief Initialise the simulation state.
 *        Call once after HAL_Init() in main().
 * @param base_timestamp  Seconds since reset (use HAL_GetTick()/1000).
 */
void fleet_data_init(uint32_t base_timestamp);

/**
 * @brief Advance simulation by one second.
 *        Call from the 1 Hz TIM2 tick in the main loop.
 */
void fleet_data_update(void);

/**
 * @brief Fill a telemetry frame with the current simulated sensor data.
 *
 * Writes into the caller-supplied frame buffer — never touches the
 * active I2C buffer directly (double-buffer responsibility is in main.c).
 *
 * @param frame  Output buffer to fill.
 * @param type   MSG_TRIP_START, MSG_TRIP_DATA, or MSG_TRIP_END.
 */
void fleet_get_frame(telemetry_frame_t *frame, char type);

#endif /* FLEET_DATA_H */

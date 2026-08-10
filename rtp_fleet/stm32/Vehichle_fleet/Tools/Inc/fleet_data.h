/**
 * @file fleet_data.h
 * @brief Simulated sensor data generator for 100 virtual vehicles.
 *
 * Each virtual car has its own state:
 *   - Current position (lat/lon) that moves each sample
 *   - Current speed
 *   - Trip state: IDLE, ACTIVE, or ENDING
 *
 * fleet_data_init()   — call once at startup
 * fleet_data_update() — call every second to advance all car states
 * fleet_get_frame()   — fill a telemetry_frame_t for a specific car
 */

#ifndef FLEET_DATA_H
#define FLEET_DATA_H

#include <stdint.h>
#include <stdio.h>

/* Must match fleet server common.h */
#define FLEET_MAX_CARS       100
#define FRAME_SIZE_BYTES     84

/* Message types */
#define MSG_TRIP_START  'S'
#define MSG_TRIP_DATA   'D'
#define MSG_TRIP_END    'E'

/* Trip states */
typedef enum {
    CAR_IDLE,      /* between trips — waiting to start next trip  */
    CAR_ACTIVE,    /* trip in progress — sending 'D' frames       */
    CAR_ENDING     /* sending one final 'E' frame then back IDLE  */
} car_trip_state_t;

/**
 * @brief State of one simulated vehicle.
 *        Maintained across calls to fleet_data_update().
 */
typedef struct {
    /* Identity */
    char car_id[8];      /* "CAR-001" ... "CAR-100"      */
    char driver_id[8];   /* "DRV-001" ... "DRV-100"      */
    char trip_id[20];    /* "T<unix_ts><car_num>"         */

    /* GPS position — moves slightly each second */
    float latitude;
    float longitude;
    float speed_kmh;     /* current simulated speed       */

    /* Simulated sensor readings (last computed values) */
    float accel_x;       /* m/s² forward/back             */
    float accel_y;       /* m/s² lateral                  */
    float accel_z;       /* m/s² vertical (≈ 9.81)        */
    float gyro_x;        /* °/s pitch                     */
    float gyro_y;        /* °/s roll                      */
    float gyro_z;        /* °/s yaw (turn rate)           */

    /* Trip state machine */
    car_trip_state_t trip_state;
    uint32_t         trip_sec;      /* seconds since trip start    */
    uint32_t         idle_sec;      /* seconds in idle state       */
    uint32_t         trip_duration; /* planned trip length (sec)   */
    uint32_t         idle_duration; /* planned idle length (sec)   */

    /* Pseudo-random seed per car for variation */
    uint32_t rand_seed;

} car_state_t;

/**
 * @brief Telemetry frame — 84 bytes packed.
 *        Must be binary-identical to BBG side fleet_bbg.c telemetry_frame_t.
 */
#pragma pack(push,1)
typedef struct {
    char     car_id[8];
    char     driver_id[8];
    char     trip_id[20];
    float    latitude;
    float    longitude;
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
    uint8_t  reserved[5];
} telemetry_frame_t;
#pragma pack(pop)

/* ── Public API ──────────────────────────────────────────────────────── */

/**
 * @brief Initialise the simulation state for all 100 cars.
 *        Call once at startup before the main loop.
 * @param base_timestamp  Unix timestamp to use as base (use HAL_GetTick()/1000
 *                        or a fixed value for reproducibility).
 */
void fleet_data_init(uint32_t base_timestamp);

/**
 * @brief Advance the simulation by one second for all cars.
 *        Call from the 1 Hz TIM2 timer tick in the main loop.
 */
void fleet_data_update(void);

/**
 * @brief Fill a telemetry frame for car at index idx (0-99).
 * @param idx    Car index 0 to FLEET_MAX_CARS-1.
 * @param frame  Output frame to fill.
 */
void fleet_get_frame(int idx, telemetry_frame_t *frame);

/**
 * @brief Get the message type for the current state of car idx.
 * @return MSG_TRIP_START, MSG_TRIP_DATA, or MSG_TRIP_END.
 */
char fleet_get_msg_type(int idx);

#endif /* FLEET_DATA_H */

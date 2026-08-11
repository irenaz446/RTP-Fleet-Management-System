/**
 * @file fleet_data.c
 * @brief Sensor simulation for one vehicle — V2 single car.
 *
 * Simulates realistic accelerometer, gyroscope, and GPS data
 * (including altitude) for one car making repeated trips.
 *
 * Random events injected:
 *   8%  chance per second: harsh braking   (accel_x < -4.5 m/s²)
 *   5%  chance per second: harsh accel     (accel_x >  3.5 m/s²)
 *   10% chance per second: harsh cornering (|gyro_z| > 50 °/s)
 *   7%  chance per second: overspeed       (gps_speed > 75 km/h)
 */

#include "fleet_data.h"
#include <string.h>
#include <stdio.h>

/* ── Base GPS position (Tel Aviv) ────────────────────────────────────── */
#define BASE_LAT        32.0853f
#define BASE_LON        34.7817f
#define BASE_ALT        30.0f

/* ── LCG pseudo-random number generator ─────────────────────────────── */
static uint32_t g_rand_seed;

static float lcg_float(float min, float max)
{
    g_rand_seed = g_rand_seed * 1664525u + 1013904223u;
    float r = (float)((g_rand_seed >> 16) & 0x7FFF) / 32767.0f;
    return min + r * (max - min);
}

static int lcg_chance(int percent)
{
    g_rand_seed = g_rand_seed * 1664525u + 1013904223u;
    return (int)((g_rand_seed >> 16) & 0x7FFF) % 100 < percent;
}

/* ── Persistent simulation state ─────────────────────────────────────── */
static float    g_lat;
static float    g_lon;
static float    g_alt;
static float    g_speed_kmh;
static uint32_t g_trip_count;
static uint32_t g_base_ts;

/* ── Public API ──────────────────────────────────────────────────────── */

void fleet_data_init(uint32_t base_timestamp)
{
    g_rand_seed  = 0xDEADBEEF ^ base_timestamp;
    g_lat        = BASE_LAT;
    g_lon        = BASE_LON;
    g_alt        = BASE_ALT;
    g_speed_kmh  = 0.0f;
    g_trip_count = 0;
    g_base_ts    = base_timestamp;
}

void fleet_data_update(void)
{
    /* Advance GPS position based on current speed */
    float dist_deg = g_speed_kmh / 3600.0f / 111.0f;
    g_lat += dist_deg * lcg_float(0.5f, 1.0f);
    g_lon += dist_deg * lcg_float(0.0f, 0.5f);

    /* Altitude drifts gently */
    g_alt += lcg_float(-1.0f, 1.0f);
    if (g_alt <   0.0f) g_alt =   0.0f;
    if (g_alt > 200.0f) g_alt = 200.0f;
}

void fleet_get_frame(telemetry_frame_t *frame, char type)
{
    memset(frame, 0, sizeof(telemetry_frame_t));

    /* ── Identity ─────────────────────────────────────────────────── */
    snprintf(frame->car_id,  sizeof(frame->car_id),  "CAR-001");
    snprintf(frame->trip_id, sizeof(frame->trip_id),
             "T%07lu%03lu",
             (unsigned long)g_base_ts,
             (unsigned long)g_trip_count);

    /* ── GPS ──────────────────────────────────────────────────────── */
    if (type == MSG_TRIP_START || type == MSG_TRIP_END) {
        g_speed_kmh = 0.0f;
        if (type == MSG_TRIP_START) {
            g_lat = BASE_LAT;
            g_lon = BASE_LON;
            g_alt = BASE_ALT;
            g_trip_count++;
        }
    } else {
        float target = lcg_chance(7)
                     ? lcg_float(75.0f, 95.0f)
                     : lcg_float(30.0f, 65.0f);
        g_speed_kmh += (target - g_speed_kmh) * 0.3f;
    }

    frame->latitude      = g_lat;
    frame->longitude     = g_lon;
    frame->altitude      = g_alt;
    frame->gps_speed_kmh = g_speed_kmh;
    frame->gps_fix       = 1;
    frame->satellites    = 8;

    /* ── Accelerometer ────────────────────────────────────────────── */
    float target_spd = lcg_float(30.0f, 65.0f);
    frame->accel_x = (target_spd - g_speed_kmh) / 3.6f * 0.3f;
    frame->accel_y = lcg_float(-0.5f, 0.5f);
    frame->accel_z = 9.81f + lcg_float(-0.3f, 0.3f);

    if (type == MSG_TRIP_DATA) {
        if (lcg_chance(8))
            frame->accel_x = lcg_float(-7.0f, -4.5f);   /* harsh brake */
        if (lcg_chance(5))
            frame->accel_x = lcg_float( 3.5f,  5.5f);   /* harsh accel */
    }

    /* ── Gyroscope ────────────────────────────────────────────────── */
    frame->gyro_x = lcg_float(-2.0f,  2.0f);
    frame->gyro_y = lcg_float(-1.0f,  1.0f);
    frame->gyro_z = lcg_float(-15.0f, 15.0f);

    if (type == MSG_TRIP_DATA && lcg_chance(10)) {
        float sign    = lcg_chance(50) ? 1.0f : -1.0f;
        frame->gyro_z  = sign * lcg_float(50.0f, 80.0f);  /* harsh turn */
        frame->accel_y = sign * lcg_float( 3.5f,  5.0f);
    }

    /* ── Control ──────────────────────────────────────────────────── */
    frame->timestamp_sec = g_base_ts + (uint32_t)(g_trip_count * 200);
    frame->msg_type      = type;
    frame->reserved[0]   = FRAME_SYNC_BYTE;
}

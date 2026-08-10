/**
 * @file fleet_data.c
 * @brief Simulated sensor data for 100 virtual vehicles.
 *
 * Each car has independent state:
 *   - Unique starting GPS position spread across Israel
 *   - Individual pseudo-random generator (LCG) for variation
 *   - Trip state machine: IDLE → ACTIVE → ENDING → IDLE
 *   - Realistic physics: smooth speed changes, position movement,
 *     occasional harsh events (braking, sharp turns, overspeed)
 *
 * No stdlib random() — uses a simple LCG per car so it works on
 * bare-metal STM32 without OS support.
 */

#include "fleet_data.h"
#include <string.h>
#include <math.h>

/* ── LCG pseudo-random number generator (per-car seed) ─────────────── */

/**
 * @brief Advance one LCG step and return a value 0..32767.
 * @param seed  Per-car seed, updated in place.
 */
static uint32_t lcg_next(uint32_t *seed)
{
    *seed = (*seed * 1664525u + 1013904223u);
    return (*seed >> 16) & 0x7FFF;
}

/**
 * @brief Return a float in [min, max] using the car's LCG.
 */
static float lcg_float(uint32_t *seed, float min, float max)
{
    float r = (float)lcg_next(seed) / 32767.0f;
    return min + r * (max - min);
}

/**
 * @brief Return 1 with probability (percent/100), 0 otherwise.
 */
static int lcg_chance(uint32_t *seed, int percent)
{
    return (int)lcg_next(seed) % 100 < percent;
}

/* ── Starting positions for 100 cars (spread across Israel) ─────────── */

/*
 * We define 10 base locations and spread 10 cars around each.
 * Each car gets a small random offset so they don't stack exactly.
 */
static const float BASE_LAT[10] = {
    32.0853f,  /* Tel Aviv       */
    31.7683f,  /* Jerusalem      */
    32.7940f,  /* Haifa          */
    31.2518f,  /* Beer Sheva     */
    29.5577f,  /* Eilat          */
    32.3215f,  /* Netanya        */
    32.1642f,  /* Herzliya       */
    31.8928f,  /* Rehovot        */
    32.4406f,  /* Hadera         */
    33.0050f   /* Nahariya       */
};

static const float BASE_LON[10] = {
    34.7818f,  /* Tel Aviv       */
    35.2137f,  /* Jerusalem      */
    34.9896f,  /* Haifa          */
    34.7913f,  /* Beer Sheva     */
    34.9519f,  /* Eilat          */
    34.8594f,  /* Netanya        */
    34.8440f,  /* Herzliya       */
    34.8112f,  /* Rehovot        */
    34.9174f,  /* Hadera         */
    35.0981f   /* Nahariya       */
};

/* ── Trip timing parameters ─────────────────────────────────────────── */
#define TRIP_DURATION_MIN_SEC   60    /* shortest trip: 1 minute          */
#define TRIP_DURATION_MAX_SEC  300    /* longest trip:  5 minutes         */
#define IDLE_DURATION_MIN_SEC   10    /* shortest wait between trips      */
#define IDLE_DURATION_MAX_SEC   30    /* longest wait between trips       */

/* ── Harsh event probabilities (% chance per second) ────────────────── */
#define PROB_HARSH_BRAKE    8    /* % chance of sudden braking per sec   */
#define PROB_HARSH_ACCEL    5    /* % chance of sudden acceleration      */
#define PROB_HARSH_TURN    10    /* % chance of sharp cornering          */
#define PROB_OVERSPEED      7    /* % chance of speeding                 */

/* ── Module state ────────────────────────────────────────────────────── */
static car_state_t g_cars[FLEET_MAX_CARS];
static uint32_t    g_base_ts = 0;
static uint32_t    g_now_ts  = 0;   /* current simulated time            */

/* ── Initialisation ─────────────────────────────────────────────────── */

void fleet_data_init(uint32_t base_timestamp)
{
    g_base_ts = base_timestamp;
    g_now_ts  = base_timestamp;

    for (int i = 0; i < FLEET_MAX_CARS; i++) {
        car_state_t *c = &g_cars[i];

        /* Identity */
        snprintf(c->car_id,    sizeof(c->car_id),    "CAR-%03d", i + 1);
        snprintf(c->driver_id, sizeof(c->driver_id), "DRV-%03d", i + 1);

        /* Unique seed per car for independent randomness */
        c->rand_seed = (uint32_t)(0xDEAD0000u + i * 0x1337u + base_timestamp);

        /* Starting position: base city + small random offset */
        int city = i / 10;   /* cars 0-9 → city 0, cars 10-19 → city 1 ... */
        c->latitude  = BASE_LAT[city] + lcg_float(&c->rand_seed, -0.05f, 0.05f);
        c->longitude = BASE_LON[city] + lcg_float(&c->rand_seed, -0.05f, 0.05f);

        /* Initial sensor values (stationary) */
        c->speed_kmh = 0.0f;
        c->accel_x   = 0.0f;
        c->accel_y   = 0.0f;
        c->accel_z   = 9.81f;
        c->gyro_x    = 0.0f;
        c->gyro_y    = 0.0f;
        c->gyro_z    = 0.0f;

        /* Stagger trip starts so not all 100 cars start simultaneously */
        c->trip_state    = CAR_IDLE;
        c->idle_sec      = 0;
        c->trip_sec      = 0;
        /* Stagger idle duration: car 0 starts after 0s, car 99 after ~99s */
        c->idle_duration = (uint32_t)(i % 20);
        c->trip_duration = (uint32_t)lcg_float(&c->rand_seed,
                                               TRIP_DURATION_MIN_SEC,
                                               TRIP_DURATION_MAX_SEC);
    }
}

/* ── Per-car state update (called every second) ─────────────────────── */

/**
 * @brief Update the state of one car for the current second.
 *        Advances trip state machine and updates all sensor values.
 */
static void update_car(car_state_t *c)
{
    switch (c->trip_state) {

        /* ── IDLE: waiting for next trip ─────────────────────────────── */
        case CAR_IDLE:
            c->idle_sec++;
            c->speed_kmh = 0.0f;
            c->accel_x   = 0.0f;
            c->accel_y   = 0.0f;
            c->accel_z   = 9.81f;
            c->gyro_z    = 0.0f;

            if (c->idle_sec >= c->idle_duration) {
                /* Start a new trip */
                c->trip_state = CAR_ACTIVE;
                c->trip_sec   = 0;
                c->idle_sec   = 0;
                snprintf(c->trip_id, sizeof(c->trip_id),
                         "T%07lu%03d", g_now_ts % 9999999,
                         (int)(c - g_cars) + 1);
            }
            break;

        /* ── ACTIVE: driving — generate realistic sensor data ─────────── */
        case CAR_ACTIVE:
            c->trip_sec++;

            /* Normal driving: speed 30-70 km/h */
            float target_speed = lcg_float(&c->rand_seed, 30.0f, 65.0f);

            /* Occasional overspeed event */
            if (lcg_chance(&c->rand_seed, PROB_OVERSPEED))
                target_speed = lcg_float(&c->rand_seed, 75.0f, 95.0f);

            /* Smooth speed change toward target */
            float speed_delta = target_speed - c->speed_kmh;
            c->speed_kmh += speed_delta * 0.3f;

            /* Normal forward acceleration (speed change in m/s²) */
            c->accel_x = (speed_delta / 3.6f) * 0.3f;

            /* Harsh braking event — override accel_x */
            if (lcg_chance(&c->rand_seed, PROB_HARSH_BRAKE))
                c->accel_x = lcg_float(&c->rand_seed, -7.0f, -4.5f);

            /* Harsh acceleration event */
            if (lcg_chance(&c->rand_seed, PROB_HARSH_ACCEL))
                c->accel_x = lcg_float(&c->rand_seed, 3.5f, 5.5f);

            /* Lateral acceleration: small normally, large on harsh turn */
            c->accel_y = lcg_float(&c->rand_seed, -0.5f, 0.5f);

            /* Vertical: gravity + small road vibration */
            c->accel_z = 9.81f + lcg_float(&c->rand_seed, -0.3f, 0.3f);

            /* Gyro yaw: normal turn rate */
            c->gyro_z = lcg_float(&c->rand_seed, -15.0f, 15.0f);

            /* Harsh cornering event */
            if (lcg_chance(&c->rand_seed, PROB_HARSH_TURN)) {
                float sign = lcg_chance(&c->rand_seed, 50) ? 1.0f : -1.0f;
                c->gyro_z  = sign * lcg_float(&c->rand_seed, 50.0f, 80.0f);
                c->accel_y = sign * lcg_float(&c->rand_seed, 3.5f, 5.0f);
            }

            /* Small pitch and roll from road surface */
            c->gyro_x = lcg_float(&c->rand_seed, -2.0f, 2.0f);
            c->gyro_y = lcg_float(&c->rand_seed, -1.0f, 1.0f);

            /* Move GPS position based on speed and heading */
            /* Simple: move mostly north-east at random heading */
            float heading_rad = lcg_float(&c->rand_seed, 0.0f, 6.28f);
            float dist_deg    = c->speed_kmh / 3600.0f / 111.0f;
            c->latitude  += dist_deg * cosf(heading_rad);
            c->longitude += dist_deg * sinf(heading_rad);

            /* Check if trip duration reached */
            if (c->trip_sec >= c->trip_duration)
                c->trip_state = CAR_ENDING;

            break;

        /* ── ENDING: send one final E frame then go idle ─────────────── */
        case CAR_ENDING:
            /* Decelerate to stop */
            c->speed_kmh = 0.0f;
            c->accel_x   = -1.5f;   /* gentle braking to stop */
            c->accel_y   = 0.0f;
            c->accel_z   = 9.81f;
            c->gyro_z    = 0.0f;

            /* Transition to IDLE */
            c->trip_state    = CAR_IDLE;
            c->idle_sec      = 0;
            c->trip_sec      = 0;
            c->idle_duration = (uint32_t)lcg_float(&c->rand_seed,
                                                   IDLE_DURATION_MIN_SEC,
                                                   IDLE_DURATION_MAX_SEC);
            c->trip_duration = (uint32_t)lcg_float(&c->rand_seed,
                                                   TRIP_DURATION_MIN_SEC,
                                                   TRIP_DURATION_MAX_SEC);
            break;
    }
}

/* ── Public API ─────────────────────────────────────────────────────── */

void fleet_data_update(void)
{
    g_now_ts++;
    for (int i = 0; i < FLEET_MAX_CARS; i++)
        update_car(&g_cars[i]);
}

void fleet_get_frame(int idx, telemetry_frame_t *frame)
{
    if (idx < 0 || idx >= FLEET_MAX_CARS) return;
    const car_state_t *c = &g_cars[idx];

    memset(frame, 0, sizeof(telemetry_frame_t));

    /* Identity */
    snprintf(frame->car_id,    sizeof(frame->car_id),    "%s", c->car_id);
    snprintf(frame->driver_id, sizeof(frame->driver_id), "%s", c->driver_id);
    snprintf(frame->trip_id,   sizeof(frame->trip_id),   "%s", c->trip_id);

    /* GPS */
    frame->latitude      = c->latitude;
    frame->longitude     = c->longitude;
    frame->gps_speed_kmh = c->speed_kmh;

    /* Accelerometer */
    frame->accel_x = c->accel_x;
    frame->accel_y = c->accel_y;
    frame->accel_z = c->accel_z;

    /* Gyroscope */
    frame->gyro_x = c->gyro_x;
    frame->gyro_y = c->gyro_y;
    frame->gyro_z = c->gyro_z;

    /* Control */
    frame->timestamp_sec = g_now_ts;
    frame->msg_type      = fleet_get_msg_type(idx);
    frame->gps_fix       = 1;
    frame->satellites    = 8;

    /* sync byte — fixed magic value 0xAB */
    frame->reserved[0] = 0xAB;   /* sync marker for BBG frame alignment */
}

char fleet_get_msg_type(int idx)
{
    if (idx < 0 || idx >= FLEET_MAX_CARS) return MSG_TRIP_DATA;

    switch (g_cars[idx].trip_state) {
        case CAR_IDLE:
            /* In IDLE we still need to send something — send last E frame */
            return MSG_TRIP_END;
        case CAR_ACTIVE:
            /* First second of trip → START, rest → DATA */
            return (g_cars[idx].trip_sec == 1) ? MSG_TRIP_START
                                                : MSG_TRIP_DATA;
        case CAR_ENDING:
            return MSG_TRIP_END;
        default:
            return MSG_TRIP_DATA;
    }
}

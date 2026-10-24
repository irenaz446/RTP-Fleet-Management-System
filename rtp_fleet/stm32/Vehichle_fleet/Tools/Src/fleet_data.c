/**
 * @file fleet_data.c
 * @brief Vehicle Fleet Management System V3 — real-sensor implementation.
 *
 *  Replaces the LCG simulation in V2 with:
 *    • MPU-6050  on I2C1  → accel_x/y/z, gyro_x/y/z
 *    • NEO-6M/8M on USART1 → lat, lon, alt, speed, fix, satellites
 *
 *  All other logic (frame layout, car_id, trip_id format, double-buffer
 *  protocol with BBG) is IDENTICAL to V2.
 *
 *  Target: STM32F756ZG, stm32f7xx_hal, HSI 16 MHz
 */

#include "fleet_data.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>   /* strtof, strtol */

/* ─────────────────────────────────────────────────────────────────────
 *  Module-private HAL handles (set by fleet_sensor_init)
 * ───────────────────────────────────────────────────────────────────── */
static I2C_HandleTypeDef  *s_hi2c;
static UART_HandleTypeDef *s_huart;

/* ─────────────────────────────────────────────────────────────────────
 *  GPS ring-buffer  (written in ISR, drained in fleet_data_update)
 * ───────────────────────────────────────────────────────────────────── */
volatile uint8_t  g_gps_rx_buf[GPS_BUF_SIZE];
volatile uint16_t g_gps_head;   /* ISR writes here  */
volatile uint16_t g_gps_tail;   /* main loop reads  */
volatile uint8_t  g_gps_byte;   /* single-byte DMA target */

/* ─────────────────────────────────────────────────────────────────────
 *  Latest sensor readings — updated by fleet_data_update()
 * ───────────────────────────────────────────────────────────────────── */
static float    s_accel_x, s_accel_y, s_accel_z;   /* m/s²  */
static float    s_gyro_x,  s_gyro_y,  s_gyro_z;    /* °/s   */
static float    s_lat, s_lon, s_alt;                /* deg / m */
static float    s_speed_kmh;
static uint8_t  s_gps_fix;
static uint8_t  s_satellites;

/* Trip identity (same format as V2 simulation) */
static uint32_t s_base_ts;
static uint32_t s_trip_count;

/* ─────────────────────────────────────────────────────────────────────
 *  UART ISR callback — pushes byte into ring-buffer
 *  (called by HAL whenever HAL_UART_Receive_IT completes 1 byte)
 * ───────────────────────────────────────────────────────────────────── */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != s_huart->Instance)
        return;

    uint16_t next = (g_gps_head + 1u) & GPS_BUF_MASK;
    if (next != g_gps_tail) {               /* drop byte if buffer full */
        g_gps_rx_buf[g_gps_head] = g_gps_byte;
        g_gps_head = next;
    }
    /* Re-arm for the next byte */
    HAL_UART_Receive_IT(s_huart, (uint8_t *)&g_gps_byte, 1);
}

/* ─────────────────────────────────────────────────────────────────────
 *  NMEA helpers
 * ───────────────────────────────────────────────────────────────────── */

/** Validate XOR checksum: sentence ends with *XX where XX is hex. */
static int nmea_checksum_ok(const char *sentence)
{
    /* sentence starts with '$' */
    const char *p = sentence + 1;
    uint8_t cs = 0;
    while (*p && *p != '*')
        cs ^= (uint8_t)*p++;
    if (*p != '*') return 0;
    unsigned int ref = 0;
    if (sscanf(p + 1, "%2X", &ref) != 1) return 0;
    return cs == (uint8_t)ref;
}

/**
 * Convert NMEA ddmm.mmmmm / dddmm.mmmmm + direction to decimal degrees.
 * @param raw   e.g. "3204.6000"  (lat) or "03445.3500" (lon)
 * @param dir   'N','S','E','W'
 */
static float nmea_to_dd(const char *raw, char dir)
{
    float val = strtof(raw, NULL);
    int   deg = (int)(val / 100);
    float min = val - deg * 100.0f;
    float dd  = deg + min / 60.0f;
    if (dir == 'S' || dir == 'W') dd = -dd;
    return dd;
}

/**
 * Return pointer to the n-th comma-separated field in a NMEA sentence.
 * Field 0 is the talker+message-id ("$GPGGA").
 */
static const char *nmea_field(const char *sentence, int n)
{
    const char *p = sentence;
    while (n-- > 0) {
        p = strchr(p, ',');
        if (!p) return NULL;
        p++;
    }
    return p;
}

/**
 * Copy field n from sentence into out[size].
 * Returns 1 on success, 0 if field missing or empty.
 */
static int nmea_copy_field(const char *sentence, int n,
                           char *out, size_t size)
{
    const char *p = nmea_field(sentence, n);
    if (!p || *p == ',' || *p == '*' || *p == '\0') {
        out[0] = '\0';
        return 0;
    }
    size_t i = 0;
    while (i < size - 1 && p[i] != ',' && p[i] != '*' && p[i] != '\0')
        out[i] = p[i++];
    out[i] = '\0';
    return i > 0;
}

/* ─────────────────────────────────────────────────────────────────────
 *  Parse $GPGGA — lat, lon, fix, satellites, altitude
 *
 *  $GPGGA,HHMMSS.ss,LLLL.LL,a,YYYYY.YY,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 *  Field:  0         1        2 3         4 5 6  7   8   9
 * ───────────────────────────────────────────────────────────────────── */
static void parse_gpgga(const char *s)
{
    char lat[16], lat_d[4], lon[16], lon_d[4], fix[4], sats[4], alt[16];

    if (!nmea_copy_field(s, 1, lat,   sizeof(lat)))   return;
    if (!nmea_copy_field(s, 2, lat_d, sizeof(lat_d))) return;
    if (!nmea_copy_field(s, 3, lon,   sizeof(lon)))   return;
    if (!nmea_copy_field(s, 4, lon_d, sizeof(lon_d))) return;
    if (!nmea_copy_field(s, 5, fix,   sizeof(fix)))   return;
    if (!nmea_copy_field(s, 6, sats,  sizeof(sats)))  return;
    if (!nmea_copy_field(s, 8, alt,   sizeof(alt)))   return;

    s_lat        = nmea_to_dd(lat, lat_d[0]);
    s_lon        = nmea_to_dd(lon, lon_d[0]);
    s_alt        = strtof(alt, NULL);
    s_gps_fix    = (uint8_t)atoi(fix);
    s_satellites = (uint8_t)atoi(sats);
}

/* ─────────────────────────────────────────────────────────────────────
 *  Parse $GPRMC — speed (knots → km/h)
 *
 *  $GPRMC,HHMMSS.ss,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,...*hh
 *  Field:  0         1 2        3 4         5 6   7   8
 * ───────────────────────────────────────────────────────────────────── */
static void parse_gprmc(const char *s)
{
    char status[4], lat[16], lat_d[4], lon[16], lon_d[4], spd[16];

    if (!nmea_copy_field(s, 2, status, sizeof(status))) return;
    if (status[0] != 'A') return;   /* void fix */

    if (!nmea_copy_field(s, 3, lat,   sizeof(lat)))   return;
    if (!nmea_copy_field(s, 4, lat_d, sizeof(lat_d))) return;
    if (!nmea_copy_field(s, 5, lon,   sizeof(lon)))   return;
    if (!nmea_copy_field(s, 6, lon_d, sizeof(lon_d))) return;
    if (!nmea_copy_field(s, 7, spd,   sizeof(spd)))   return;

    s_lat         = nmea_to_dd(lat, lat_d[0]);
    s_lon         = nmea_to_dd(lon, lon_d[0]);
    s_speed_kmh   = strtof(spd, NULL) * 1.852f;  /* knots → km/h */
}

/* ─────────────────────────────────────────────────────────────────────
 *  GPS ring-buffer drain — reassemble NMEA sentences and dispatch
 * ───────────────────────────────────────────────────────────────────── */
#define NMEA_LINE_MAX 100

static char   s_nmea_line[NMEA_LINE_MAX];
static uint8_t s_nmea_pos;

static void gps_drain(void)
{
    while (g_gps_tail != g_gps_head) {
        uint8_t c = g_gps_rx_buf[g_gps_tail];
        g_gps_tail = (g_gps_tail + 1u) & GPS_BUF_MASK;

        if (c == '$') {
            s_nmea_pos = 0;
        }

        if (s_nmea_pos < NMEA_LINE_MAX - 1) {
            s_nmea_line[s_nmea_pos++] = (char)c;
            s_nmea_line[s_nmea_pos]   = '\0';
        }

        if (c == '\n' && s_nmea_pos > 6) {
            /* Complete sentence: validate checksum then dispatch */
            if (nmea_checksum_ok(s_nmea_line)) {
                if (strncmp(s_nmea_line, "$GPGGA", 6) == 0)
                    parse_gpgga(s_nmea_line);
                else if (strncmp(s_nmea_line, "$GPRMC", 6) == 0)
                    parse_gprmc(s_nmea_line);
            }
            s_nmea_pos = 0;
        }
    }
}

/* ─────────────────────────────────────────────────────────────────────
 *  MPU-6050 read — 14-byte burst from reg 0x3B
 *  Byte order: AX_H AX_L AY_H AY_L AZ_H AZ_L T_H T_L GX_H GX_L GY_H GY_L GZ_H GZ_L
 * ───────────────────────────────────────────────────────────────────── */
static void imu_read(void)
{
    uint8_t raw[14];
    if (HAL_I2C_Mem_Read(s_hi2c,
                          MPU6050_ADDR_W,          /* 8-bit address = 0xD0 */
                          MPU6050_ACCEL_XOUT,       /* reg 0x3B              */
                          I2C_MEMADD_SIZE_8BIT,
                          raw, sizeof(raw),
                          10) != HAL_OK)
    {
        /* Sensor not responding — keep previous values */
        return;
    }

    int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
    /* raw[6..7] = temperature — skip */
    int16_t gx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    /* Convert: accel → m/s²,  gyro → °/s */
    s_accel_x = ax / MPU6050_ACCEL_SCALE * 9.81f;
    s_accel_y = ay / MPU6050_ACCEL_SCALE * 9.81f;
    s_accel_z = az / MPU6050_ACCEL_SCALE * 9.81f;
    s_gyro_x  = gx / MPU6050_GYRO_SCALE;
    s_gyro_y  = gy / MPU6050_GYRO_SCALE;
    s_gyro_z  = gz / MPU6050_GYRO_SCALE;
}

/* ─────────────────────────────────────────────────────────────────────
 *  Public API
 * ───────────────────────────────────────────────────────────────────── */

void fleet_sensor_init(I2C_HandleTypeDef  *hi2c_imu,
                       UART_HandleTypeDef *huart_gps)
{
    s_hi2c  = hi2c_imu;
    s_huart = huart_gps;

    /* ── Wake MPU-6050: clear sleep bit in PWR_MGMT_1 (reg 0x6B) ─── */
    uint8_t wake = 0x00;
    HAL_I2C_Mem_Write(s_hi2c,
                      MPU6050_ADDR_W,
                      MPU6050_PWR_MGMT_1,
                      I2C_MEMADD_SIZE_8BIT,
                      &wake, 1, 10);
    HAL_Delay(100);   /* sensor startup time */

    /* ── Arm GPS UART interrupt for first byte ───────────────────── */
    g_gps_head = 0;
    g_gps_tail = 0;
    HAL_UART_Receive_IT(s_huart, (uint8_t *)&g_gps_byte, 1);

    /* ── Trip identity ───────────────────────────────────────────── */
    s_base_ts    = HAL_GetTick() / 1000UL;
    s_trip_count = 0;

    /* ── Safe initial values ─────────────────────────────────────── */
    s_accel_x    = 0.0f;
    s_accel_y    = 0.0f;
    s_accel_z    = 9.81f;
    s_gyro_x     = s_gyro_y = s_gyro_z = 0.0f;
    s_lat        = 0.0f;
    s_lon        = 0.0f;
    s_alt        = 0.0f;
    s_speed_kmh  = 0.0f;
    s_gps_fix    = 0;
    s_satellites = 0;

    printf("[fleet] V3 sensor init OK  (I2C1 MPU-6050, USART1 GPS)\r\n");
}

void fleet_data_update(void)
{
    imu_read();    /* blocking I2C burst read, ~1 ms */
    gps_drain();   /* drain UART ring-buffer, parse NMEA */
}

void fleet_get_frame(telemetry_frame_t *frame, char type)
{
    memset(frame, 0, sizeof(telemetry_frame_t));

    /* ── Identity (same format as V2 simulation) ─────────────────── */
    snprintf(frame->car_id,  sizeof(frame->car_id),  "CAR-001");
    snprintf(frame->trip_id, sizeof(frame->trip_id),
             "T%07lu%03lu",
             (unsigned long)s_base_ts,
             (unsigned long)s_trip_count);

    /* ── Trip management ────────────────────────────────────────── */
    if (type == MSG_TRIP_START) {
        s_trip_count++;
    }

    /* ── IMU ─────────────────────────────────────────────────────── */
    frame->accel_x = s_accel_x;
    frame->accel_y = s_accel_y;
    frame->accel_z = s_accel_z;
    frame->gyro_x  = s_gyro_x;
    frame->gyro_y  = s_gyro_y;
    frame->gyro_z  = s_gyro_z;

    /* ── GPS ─────────────────────────────────────────────────────── */
    frame->latitude      = s_lat;
    frame->longitude     = s_lon;
    frame->altitude      = s_alt;
    frame->gps_speed_kmh = s_speed_kmh;
    frame->gps_fix       = s_gps_fix;
    frame->satellites    = s_satellites;

    /* ── Control ─────────────────────────────────────────────────── */
    frame->timestamp_sec = HAL_GetTick() / 1000UL;
    frame->msg_type      = (uint8_t)type;
    frame->reserved[0]   = FRAME_SYNC_BYTE;
}

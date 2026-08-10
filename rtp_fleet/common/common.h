/**
 * @file common.h
 * @brief Shared definitions for the Vehicle Fleet RTP telemetry system.
 *
 * Included by C (BBG) and C++ (server) components.
 */

#ifndef RTP_FLEET_COMMON_H
#define RTP_FLEET_COMMON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Network configuration ───────────────────────────────────────────── */
#define RTP_SERVER_PORT     5004       /* UDP port server listens on      */
#define RTP_PAYLOAD_TYPE    96         /* dynamic PT for telemetry        */
#define RTP_SSRC_CAR001     0x00000001 /* SSRC = car identifier           */

/* ── Packet sizes ────────────────────────────────────────────────────── */
#define RTP_HEADER_SIZE     12         /* fixed RTP header bytes          */
#define TELEMETRY_FRAME_SIZE 84        /* telemetry_frame_t size          */
#define RTP_PACKET_SIZE     (RTP_HEADER_SIZE + TELEMETRY_FRAME_SIZE) /* 96 */

/* ── Wire message type bytes ─────────────────────────────────────────── */
#define MSG_TRIP_START      'S'
#define MSG_TRIP_DATA       'D'
#define MSG_TRIP_END        'E'

/* ── Driving quality thresholds ─────────────────────────────────────── */
#define HARSH_BRAKE_THRESHOLD   -4.0f
#define HARSH_ACCEL_THRESHOLD    3.0f
#define HARSH_TURN_THRESHOLD    45.0f
#define SPEED_LIMIT_KMH         60.0f

/* ── Score penalty weights ───────────────────────────────────────────── */
#define PENALTY_HARSH_BRAKE    5
#define PENALTY_HARSH_ACCEL    3
#define PENALTY_HARSH_TURN     4
#define PENALTY_OVERSPEED      2
#define PENALTY_LOST_PACKET    1

/* ── Default paths ───────────────────────────────────────────────────── */
#define DEFAULT_DB_PATH     "rtp_fleet.db"
#define DEFAULT_LOG_DIR     "/tmp/rtp_fleet_logs"
#define DEFAULT_ALERT_LOG   "/tmp/rtp_fleet_alerts.log"

/* ─────────────────────────────────────────────────────────────────────
 * RTP Header — RFC 3550, 12 bytes, big-endian on wire.
 *
 * For this system:
 *   V   = 2  (RTP version)
 *   P   = 0  (no padding)
 *   X   = 0  (no extension)
 *   CC  = 0  (no CSRC)
 *   M   = 0  (no marker)
 *   PT  = 96 (dynamic payload type for telemetry)
 *   seq     increments per packet — server detects loss via gaps
 *   ts      seconds since trip start — reconstructs sample timing
 *   ssrc    unique car stream ID (0x00000001 for CAR-001)
 * ───────────────────────────────────────────────────────────────────── */
#pragma pack(push, 1)
typedef struct {
    uint8_t  first;     /* V=2, P, X, CC                    byte 0  */
    uint8_t  second;    /* M, PT=96                         byte 1  */
    uint16_t seq;       /* sequence number (big-endian)     bytes 2-3 */
    uint32_t timestamp; /* sample timestamp (big-endian)    bytes 4-7 */
    uint32_t ssrc;      /* synchronisation source           bytes 8-11*/
} rtp_header_t;         /* total = 12 bytes                          */
#pragma pack(pop)

/* ─────────────────────────────────────────────────────────────────────
 * Telemetry frame — 84 bytes, natural compiler layout.
 * Identical on STM32, BBG, and server.
 *
 * Changes from V1:
 *   - driver_id removed (one car = one driver, redundant)
 *   - altitude added as third GPS parameter (lat, lon, alt)
 *   - reserved expanded from [5] to [9] to keep total = 84 bytes
 *   - msg_type moved to offset 72 (was 76)
 *
 * Layout:
 *   [0-7]   car_id[8]
 *   [8-27]  trip_id[20]
 *   [28-31] latitude        float, degrees
 *   [32-35] longitude       float, degrees
 *   [36-39] altitude        float, metres above sea level
 *   [40-43] gps_speed_kmh   float, km/h
 *   [44-47] accel_x         float, m/s² forward
 *   [48-51] accel_y         float, m/s² lateral
 *   [52-55] accel_z         float, m/s² vertical
 *   [56-59] gyro_x          float, °/s pitch
 *   [60-63] gyro_y          float, °/s roll
 *   [64-67] gyro_z          float, °/s yaw
 *   [68-71] timestamp_sec   uint32
 *   [72]    msg_type        'S','D','E'
 *   [73]    gps_fix         0=none 1=fix
 *   [74]    satellites      count
 *   [75-83] reserved[9]     reserved[0]=0xAB sync marker
 *   Total = 84 bytes
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
    char     car_id[8];       /* [0-7]   "CAR-001"          */
    char     trip_id[20];     /* [8-27]  "T1234567001"      */
    float    latitude;        /* [28-31] degrees            */
    float    longitude;       /* [32-35] degrees            */
    float    altitude;        /* [36-39] metres ASL         */
    float    gps_speed_kmh;   /* [40-43] km/h               */
    float    accel_x;         /* [44-47] m/s² forward       */
    float    accel_y;         /* [48-51] m/s² lateral       */
    float    accel_z;         /* [52-55] m/s² vertical      */
    float    gyro_x;          /* [56-59] °/s pitch          */
    float    gyro_y;          /* [60-63] °/s roll           */
    float    gyro_z;          /* [64-67] °/s yaw            */
    uint32_t timestamp_sec;   /* [68-71]                    */
    uint8_t  msg_type;        /* [72]    'S','D','E'        */
    uint8_t  gps_fix;         /* [73]                       */
    uint8_t  satellites;      /* [74]                       */
    uint8_t  reserved[9];     /* [75-83] [0]=0xAB sync      */
} telemetry_frame_t;          /* total = 84 bytes           */

/**
 * @brief Driving quality metrics computed from one telemetry frame.
 */
typedef struct {
    int harsh_brake;
    int harsh_accel;
    int harsh_turn;
    int overspeed;
} metrics_t;

#ifdef __cplusplus
}
#endif

#endif /* RTP_FLEET_COMMON_H */

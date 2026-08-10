/**
 * @file TelemetryParser.hpp
 * @brief Converts a raw telemetry_frame_t into a TelemetrySample.
 *
 * In the RTP system the frame arrives as a binary struct (not a text wire
 * message as in V1). This parser validates the frame and extracts the
 * metrics computed on the BBG side.
 */

#pragma once

#include <string>
#include <optional>
#include "../common/common.h"

/** @brief Parsed telemetry sample used by TripManager and ViolationDetector. */
struct TelemetrySample {
    char     msg_type;
    std::string car_id;
    std::string trip_id;
    float    latitude;
    float    longitude;
    float    altitude;         /* metres above sea level */
    float    gps_speed_kmh;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
    uint32_t timestamp_sec;
    /* Metrics computed on BBG */
    int      harsh_brake;
    int      harsh_accel;
    int      harsh_turn;
    int      overspeed;
    /* RTP metadata */
    uint16_t rtp_seq;
    int      rtp_lost;     /* packets lost before this one */
};

class TelemetryParser {
public:
    /**
     * @brief Convert a raw telemetry_frame_t to a TelemetrySample.
     *        Computes metrics from raw sensor values.
     *
     * @param f     The raw frame received in the RTP payload.
     * @param seq   RTP sequence number for this packet.
     * @param lost  Number of packets lost before this one.
     * @return Populated TelemetrySample, or nullopt if frame invalid.
     */
    static std::optional<TelemetrySample> parse(const telemetry_frame_t &f,
                                                 uint16_t seq,
                                                 int lost)
    {
        /* Validate msg_type */
        if (f.msg_type != MSG_TRIP_START &&
            f.msg_type != MSG_TRIP_DATA  &&
            f.msg_type != MSG_TRIP_END)
            return std::nullopt;

        TelemetrySample s{};
        s.msg_type      = f.msg_type;
        s.car_id        = std::string(f.car_id);
        s.trip_id       = std::string(f.trip_id);
        s.latitude      = f.latitude;
        s.longitude     = f.longitude;
        s.altitude      = f.altitude;
        s.gps_speed_kmh = f.gps_speed_kmh;
        s.accel_x       = f.accel_x;
        s.accel_y       = f.accel_y;
        s.accel_z       = f.accel_z;
        s.gyro_x        = f.gyro_x;
        s.gyro_y        = f.gyro_y;
        s.gyro_z        = f.gyro_z;
        s.timestamp_sec = f.timestamp_sec;
        s.rtp_seq       = seq;
        s.rtp_lost      = lost;

        /* Compute driving metrics from raw sensor data */
        s.harsh_brake = (f.accel_x < HARSH_BRAKE_THRESHOLD) ? 1 : 0;
        s.harsh_accel = (f.accel_x > HARSH_ACCEL_THRESHOLD) ? 1 : 0;
        s.harsh_turn  = (f.gyro_z  >  HARSH_TURN_THRESHOLD ||
                         f.gyro_z  < -HARSH_TURN_THRESHOLD) ? 1 : 0;
        s.overspeed   = (f.gps_speed_kmh > SPEED_LIMIT_KMH) ? 1 : 0;

        return s;
    }
};

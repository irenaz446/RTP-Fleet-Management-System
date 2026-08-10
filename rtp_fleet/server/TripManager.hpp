/**
 * @file TripManager.hpp
 * @brief Tracks active trips per car and accumulates telemetry samples.
 *
 * One ActiveTrip object exists per car that has sent a START ('S') message
 * but not yet an END ('E'). Each incoming DATA ('D') sample is added to the
 * trip. When END arrives, the trip is finalised and returned for scoring.
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <ctime>
#include <cmath>
#include "TelemetryParser.hpp"

/** @brief Accumulates all data for one trip. */
struct ActiveTrip {
    std::string trip_id;
    std::string car_id;
    std::string start_time;
    uint32_t    start_ts    = 0;
    uint32_t    end_ts      = 0;

    /* Accumulated event counts */
    int   harsh_brake_count = 0;
    int   harsh_accel_count = 0;
    int   harsh_turn_count  = 0;
    int   overspeed_count   = 0;
    float max_speed_kmh     = 0.0f;

    /* For distance calculation */
    float last_lat = 0.0f;
    float last_lon = 0.0f;
    float distance_km = 0.0f;

    /* Number of samples received */
    int   sample_count = 0;
};

/** @brief Finalised trip record handed to TripQueue for DB persistence. */
struct CompletedTrip {
    std::string trip_id;
    std::string car_id;
    std::string start_time;
    std::string end_time;
    float       distance_km;
    int         score;
    std::string grade;
    int         harsh_brake_count;
    int         harsh_accel_count;
    int         harsh_turn_count;
    int         overspeed_count;
    float       max_speed_kmh;
    int         sample_count;
};

class TripManager {
public:
    /**
     * @brief Handle one incoming telemetry sample.
     *
     * MSG_TRIP_START  → create new ActiveTrip for this car
     * MSG_TRIP_DATA   → accumulate into existing trip
     * MSG_TRIP_END    → finalise and return CompletedTrip
     *
     * @return CompletedTrip if the trip just ended, std::nullopt otherwise.
     */
    std::optional<CompletedTrip> process(const TelemetrySample &s)
    {
        std::lock_guard<std::mutex> lock(mx_);

        if (s.msg_type == MSG_TRIP_START) {
            ActiveTrip t;
            t.trip_id   = s.trip_id;
            t.car_id    = s.car_id;
                t.start_ts  = s.timestamp_sec;
            t.start_time = timeStr(s.timestamp_sec);
            t.last_lat  = s.latitude;
            t.last_lon  = s.longitude;
            trips_[s.car_id] = t;
            return std::nullopt;
        }

        auto it = trips_.find(s.car_id);
        if (it == trips_.end()) return std::nullopt;

        ActiveTrip &t = it->second;

        /* Accumulate events */
        t.harsh_brake_count += s.harsh_brake;
        t.harsh_accel_count += s.harsh_accel;
        t.harsh_turn_count  += s.harsh_turn;
        t.overspeed_count   += s.overspeed;
        if (s.gps_speed_kmh > t.max_speed_kmh)
            t.max_speed_kmh = s.gps_speed_kmh;

        /* Accumulate distance (simple flat-earth approximation) */
        float dlat = s.latitude  - t.last_lat;
        float dlon = s.longitude - t.last_lon;
        t.distance_km += std::sqrt(dlat*dlat + dlon*dlon) * 111.0f;
        t.last_lat = s.latitude;
        t.last_lon = s.longitude;
        t.sample_count++;

        if (s.msg_type == MSG_TRIP_DATA) return std::nullopt;

        /* MSG_TRIP_END — finalise */
        t.end_ts = s.timestamp_sec;
        CompletedTrip c = finalise(t);
        trips_.erase(it);
        return c;
    }

    /** @brief Number of currently active trips. */
    std::size_t activeCount() const
    {
        std::lock_guard<std::mutex> lock(mx_);
        return trips_.size();
    }

private:
    mutable std::mutex mx_;
    std::unordered_map<std::string, ActiveTrip> trips_;

    static CompletedTrip finalise(const ActiveTrip &t)
    {
        CompletedTrip c;
        c.trip_id            = t.trip_id;
        c.car_id             = t.car_id;
        c.start_time         = t.start_time;
        c.end_time           = timeStr(t.end_ts);
        c.distance_km        = t.distance_km;
        c.harsh_brake_count  = t.harsh_brake_count;
        c.harsh_accel_count  = t.harsh_accel_count;
        c.harsh_turn_count   = t.harsh_turn_count;
        c.overspeed_count    = t.overspeed_count;
        c.max_speed_kmh      = t.max_speed_kmh;
        c.sample_count       = t.sample_count;

        /* Score */
        int score = 100;
        score -= t.harsh_brake_count * PENALTY_HARSH_BRAKE;
        score -= t.harsh_accel_count * PENALTY_HARSH_ACCEL;
        score -= t.harsh_turn_count  * PENALTY_HARSH_TURN;
        score -= t.overspeed_count   * PENALTY_OVERSPEED;
        if (score < 0) score = 0;
        c.score = score;

        /* Grade */
        if      (score >= 90) c.grade = "Excellent";
        else if (score >= 75) c.grade = "Good";
        else if (score >= 60) c.grade = "Acceptable";
        else if (score >= 40) c.grade = "Poor";
        else                  c.grade = "Dangerous";

        return c;
    }

    static std::string timeStr(uint32_t ts)
    {
        time_t t = (time_t)ts;
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S",
                      std::localtime(&t));
        return buf;
    }
};

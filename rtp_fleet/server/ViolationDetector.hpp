/**
 * @file ViolationDetector.hpp
 * @brief Real-time driving violation detection and alert logging.
 *
 * Called from the TCP thread on every incoming DATA sample.
 * Writes alerts immediately to the alert log file so operators can
 * monitor violations in real time with: tail -f /tmp/fleet_alerts.log
 */

#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <ctime>
#include "TelemetryParser.hpp"
#include "../common/common.h"

class ViolationDetector {
public:
    explicit ViolationDetector(const std::string &alertLogPath)
        : logPath_(alertLogPath)
    {
        file_.open(logPath_, std::ios::app);
    }

    ~ViolationDetector() { if (file_.is_open()) file_.close(); }

    /**
     * @brief Check one sample for violations and log any found.
     * @param s  Parsed telemetry sample.
     */
    void check(const TelemetrySample &s)
    {
        if (s.harsh_brake)
            log("HARSH_BRAKE", s,
                std::to_string(s.accel_x) + " m/s²");

        if (s.harsh_accel)
            log("HARSH_ACCEL", s,
                std::to_string(s.accel_x) + " m/s²");

        if (s.harsh_turn)
            log("HARSH_TURN", s,
                std::to_string(s.gyro_z) + " °/s");

        if (s.overspeed)
            log("OVERSPEED", s,
                std::to_string(s.gps_speed_kmh) + " km/h");
    }

private:
    std::string   logPath_;
    std::ofstream file_;
    std::mutex    mx_;

    void log(const std::string &type,
             const TelemetrySample &s,
             const std::string &detail)
    {
        std::string ts = nowStr();
        std::string line = "[" + ts + "] "
                         + type + "\t"
                         + s.car_id + "\t"
                         + "lat=" + std::to_string(s.latitude)
                         + " lon=" + std::to_string(s.longitude)
                         + " val=" + detail + "\n";

        std::lock_guard<std::mutex> lock(mx_);
        if (file_.is_open()) {
            file_ << line;
            file_.flush();
        }
    }

    static std::string nowStr()
    {
        char buf[32];
        time_t now = time(nullptr);
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S",
                      std::localtime(&now));
        return buf;
    }
};

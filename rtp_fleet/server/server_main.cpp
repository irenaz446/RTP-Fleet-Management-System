/**
 * @file server_main.cpp
 * @brief Vehicle Fleet RTP Server — entry point.
 *
 * Two threads in one process:
 *   1. RTP thread  — UDP recvfrom() loop, receives 96-byte RTP datagrams
 *                    parses RTP header, extracts telemetry_frame_t
 *                    detects lost packets via sequence number gaps
 *                    feeds TripManager → on END pushes to TripQueue
 *                    ViolationDetector checks every sample
 *
 *   2. DB thread   — pops CompletedTrip from TripQueue → SQLite
 *
 * Configuration (config/rtp_fleet.cfg):
 *   PORT        UDP listen port     (default 5004)
 *   DB_PATH     SQLite file         (default rtp_fleet.db)
 *   LOG_FILE    Server log          (default /tmp/rtp_fleet_logs/server.log)
 *   ALERT_LOG   Violation alert log (default /tmp/rtp_fleet_alerts.log)
 */

#include <iostream>
#include <thread>
#include <atomic>
#include <csignal>
#include <sys/stat.h>
#include <memory>

#include "../common/common.h"
#include "../common/Logger.hpp"
#include "../common/Config.hpp"
#include "RtpReceiver.hpp"
#include "TelemetryParser.hpp"
#include "TripManager.hpp"
#include "TripQueue.hpp"
#include "ViolationDetector.hpp"
#include "Database.hpp"

/* ── Global shutdown flag ────────────────────────────────────────────── */
static std::atomic<bool> g_running{true};
static void sighandler(int) { g_running = false; }

/* ── Shared between RTP thread and DB thread ─────────────────────────── */
static TripQueue g_trip_queue;

/* ── DB Thread ───────────────────────────────────────────────────────── */
void db_thread_func(const std::string &db_path)
{
    Logger::info("DB thread started");
    std::unique_ptr<Database> db;
    try {
        db = std::make_unique<Database>(db_path);
    } catch (const std::exception &e) {
        Logger::error(std::string("DB open failed: ") + e.what());
        return;
    }

    while (g_running || g_trip_queue.size() > 0) {
        bool running = g_running.load();
        auto trip = g_trip_queue.pop(running);
        if (!trip.has_value()) break;
        if (trip->trip_id.empty()) continue;   /* skip shutdown sentinel */
        db->insertTrip(*trip);
    }

    Logger::info("DB thread stopped");
}

/* ── RTP Thread ──────────────────────────────────────────────────────── */
void rtp_thread_func(int port, const std::string &alert_log)
{
    Logger::info("RTP thread started on UDP port " + std::to_string(port));

    ViolationDetector detector(alert_log);
    TripManager       trip_manager;
    RtpReceiver       receiver(port);

    if (!receiver.open()) {
        Logger::error("RTP thread: failed to open UDP socket");
        g_running = false;
        return;
    }

    long total_samples = 0;
    long total_trips   = 0;
    long total_lost    = 0;

    receiver.run(g_running,
        [&](const telemetry_frame_t &frame,
            uint16_t seq, uint32_t ssrc, int lost)
        {
            total_lost += lost;

            auto sample = TelemetryParser::parse(frame, seq, lost);
            if (!sample.has_value()) {
                Logger::warn("RTP: invalid frame discarded");
                return;
            }

            total_samples++;

            /* Check for violations on data samples */
            if (sample->msg_type == MSG_TRIP_DATA)
                detector.check(*sample);

            /* Feed to trip manager */
            auto completed = trip_manager.process(*sample);
            if (completed.has_value()) {
                total_trips++;
                /* Add RTP loss to score penalty */
                completed->score = std::max(0,
                    completed->score - (int)total_lost * PENALTY_LOST_PACKET);

                Logger::info("Trip complete: " + completed->trip_id +
                             " score=" + std::to_string(completed->score) +
                             " (" + completed->grade + ")" +
                             " lost_pkts=" + std::to_string(total_lost));
                g_trip_queue.push(*completed);
                total_lost = 0;   /* reset per-trip loss counter */
            }

            if (total_samples % 60 == 0) {
                Logger::info("Status: samples=" +
                             std::to_string(total_samples) +
                             " trips=" + std::to_string(total_trips) +
                             " rtp_lost=" + std::to_string(total_lost));
            }
        });

    Logger::info("RTP thread stopped");
}

/* ── Main ────────────────────────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    const char *cfg_path = (argc > 1) ? argv[1] : "config/rtp_fleet.cfg";
    Config cfg(cfg_path);

    const int         port      = cfg.getInt("PORT",     RTP_SERVER_PORT);
    const std::string db_path   = cfg.get("DB_PATH",     DEFAULT_DB_PATH);
    const std::string log_file  = cfg.get("LOG_FILE",    "/tmp/rtp_fleet_logs/server.log");
    const std::string alert_log = cfg.get("ALERT_LOG",   DEFAULT_ALERT_LOG);

    mkdir(DEFAULT_LOG_DIR, 0755);
    Logger::init(log_file);

    std::cout << "=== Vehicle Fleet RTP Server starting ===\n";
    std::cout << "UDP port  : " << port      << "\n";
    std::cout << "Database  : " << db_path   << "\n";
    std::cout << "Alert log : " << alert_log << "\n\n";
    Logger::info("=== Vehicle Fleet RTP Server starting ===");

    signal(SIGINT,  sighandler);
    signal(SIGTERM, sighandler);

    /* Start DB thread */
    std::thread db_thread(db_thread_func, db_path);

    /* Start RTP receiver thread */
    std::thread rtp_thread(rtp_thread_func, port, alert_log);

    /* Main thread waits */
    rtp_thread.join();

    /* Wake DB thread and flush */
    g_trip_queue.push({});
    db_thread.join();

    Logger::info("Vehicle Fleet RTP Server shut down cleanly");
    Logger::close();
    return 0;
}

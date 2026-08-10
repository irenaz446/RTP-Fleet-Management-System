/**
 * @file Database.hpp
 * @brief SQLite database wrapper for the fleet management system.
 *
 * Tables: trips, violations, drivers.
 * All writes use prepared statements with bound parameters.
 * Called exclusively from the DB thread — no locking needed here.
 */

#pragma once

#include <string>
#include <stdexcept>
#include <sqlite3.h>
#include "TripManager.hpp"
#include "TelemetryParser.hpp"

class Database {
public:
    explicit Database(const std::string &path);
    ~Database();

    Database(const Database &)            = delete;
    Database &operator=(const Database &) = delete;

    /** @brief Persist a completed trip and update driver statistics. */
    void insertTrip(const CompletedTrip &trip);

    /** @brief Persist one driving violation event. */
    void insertViolation(const TelemetrySample &s,
                         const std::string &type,
                         float severity);

private:
    sqlite3 *db_ = nullptr;

    void exec(const std::string &sql);
    void createSchema();
    static std::string nowStr();
};

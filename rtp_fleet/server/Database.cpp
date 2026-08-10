/**
 * @file Database.cpp
 * @brief Fleet SQLite database implementation.
 */

#include "Database.hpp"
#include "../common/Logger.hpp"

#include <ctime>
#include <stdexcept>
#include <cstring>

Database::Database(const std::string &path)
{
    if (sqlite3_open(path.c_str(), &db_) != SQLITE_OK) {
        std::string err = db_ ? sqlite3_errmsg(db_) : "unknown";
        throw std::runtime_error("sqlite3_open('" + path + "'): " + err);
    }
    exec("PRAGMA journal_mode=WAL;");
    exec("PRAGMA synchronous=NORMAL;");
    createSchema();
    Logger::info("Fleet database opened: " + path);
}

Database::~Database()
{
    if (db_) { sqlite3_close(db_); db_ = nullptr; }
}

void Database::createSchema()
{
    exec(R"(
        CREATE TABLE IF NOT EXISTS drivers (
            car_id       TEXT PRIMARY KEY,
            total_trips  INTEGER DEFAULT 0,
            avg_score    REAL    DEFAULT 0,
            total_km     REAL    DEFAULT 0
        );
    )");

    exec(R"(
        CREATE TABLE IF NOT EXISTS trips (
            id                 INTEGER PRIMARY KEY AUTOINCREMENT,
            trip_id            TEXT UNIQUE,
            car_id             TEXT,
                    start_time         TEXT,
            end_time           TEXT,
            distance_km        REAL,
            score              INTEGER,
            grade              TEXT,
            harsh_brake_count  INTEGER,
            harsh_accel_count  INTEGER,
            harsh_turn_count   INTEGER,
            overspeed_count    INTEGER,
            max_speed_kmh      REAL,
            sample_count       INTEGER,
            recorded_at        TEXT
        );
    )");

    exec(R"(
        CREATE TABLE IF NOT EXISTS violations (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            trip_id      TEXT,
            car_id       TEXT,
            timestamp    TEXT,
            type         TEXT,
            severity     REAL,
            latitude     REAL,
            longitude    REAL
        );
    )");

    Logger::info("Fleet database schema verified");
}

void Database::insertTrip(const CompletedTrip &t)
{
    const char *sql =
        "INSERT OR REPLACE INTO trips "
        "(trip_id, car_id, start_time, end_time, "
        " distance_km, score, grade, harsh_brake_count, harsh_accel_count, "
        " harsh_turn_count, overspeed_count, max_speed_kmh, "
        " sample_count, recorded_at) "
        "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?);";

    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        Logger::error("insertTrip prepare: " +
                      std::string(sqlite3_errmsg(db_)));
        return;
    }

    std::string now = nowStr();
    sqlite3_bind_text  (stmt,  1, t.trip_id.c_str(),    -1, SQLITE_TRANSIENT);
    sqlite3_bind_text  (stmt,  2, t.car_id.c_str(),     -1, SQLITE_TRANSIENT);
    sqlite3_bind_text  (stmt,  4, t.start_time.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text  (stmt,  5, t.end_time.c_str(),   -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt,  6, t.distance_km);
    sqlite3_bind_int   (stmt,  7, t.score);
    sqlite3_bind_text  (stmt,  8, t.grade.c_str(),      -1, SQLITE_TRANSIENT);
    sqlite3_bind_int   (stmt,  9, t.harsh_brake_count);
    sqlite3_bind_int   (stmt, 10, t.harsh_accel_count);
    sqlite3_bind_int   (stmt, 11, t.harsh_turn_count);
    sqlite3_bind_int   (stmt, 12, t.overspeed_count);
    sqlite3_bind_double(stmt, 13, t.max_speed_kmh);
    sqlite3_bind_int   (stmt, 14, t.sample_count);
    sqlite3_bind_text  (stmt, 15, now.c_str(),           -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE)
        Logger::error("insertTrip step: " +
                      std::string(sqlite3_errmsg(db_)));
    else
        Logger::info("Trip saved: " + t.trip_id +
                     " score=" + std::to_string(t.score) +
                     " (" + t.grade + ")");

    sqlite3_finalize(stmt);

    /* Update driver aggregate statistics */
    const char *upd =
        "INSERT INTO drivers (car_id, total_trips, avg_score, total_km) "
        "VALUES (?, 1, ?, ?) "
        "ON CONFLICT(car_id) DO UPDATE SET "
        "  total_trips = total_trips + 1, "
        "  avg_score   = (avg_score * total_trips + excluded.avg_score) "
        "                / (total_trips + 1), "
        "  total_km    = total_km + excluded.total_km;";

    if (sqlite3_prepare_v2(db_, upd, -1, &stmt, nullptr) == SQLITE_OK) {
        sqlite3_bind_text  (stmt, 1, t.car_id.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_double(stmt, 2, t.score);
        sqlite3_bind_double(stmt, 3, t.distance_km);
        sqlite3_step(stmt);
        sqlite3_finalize(stmt);
    }
}

void Database::insertViolation(const TelemetrySample &s,
                               const std::string &type,
                               float severity)
{
    const char *sql =
        "INSERT INTO violations "
        "(trip_id, car_id, timestamp, type, severity, "
        " latitude, longitude) "
        "VALUES (?,?,?,?,?,?,?);";

    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK)
        return;

    std::string ts = nowStr();
    sqlite3_bind_text  (stmt, 1, s.trip_id.c_str(),   -1, SQLITE_TRANSIENT);
    sqlite3_bind_text  (stmt, 2, s.car_id.c_str(),    -1, SQLITE_TRANSIENT);
    sqlite3_bind_text  (stmt, 4, ts.c_str(),           -1, SQLITE_TRANSIENT);
    sqlite3_bind_text  (stmt, 5, type.c_str(),         -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 6, severity);
    sqlite3_bind_double(stmt, 7, s.latitude);
    sqlite3_bind_double(stmt, 8, s.longitude);
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);
}

void Database::exec(const std::string &sql)
{
    char *err = nullptr;
    if (sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err) != SQLITE_OK) {
        std::string msg = err ? err : "unknown";
        sqlite3_free(err);
        throw std::runtime_error("SQL: " + msg);
    }
}

std::string Database::nowStr()
{
    char buf[32];
    time_t now = time(nullptr);
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S",
                  std::localtime(&now));
    return buf;
}

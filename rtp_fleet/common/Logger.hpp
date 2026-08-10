/**
 * @file Logger.hpp
 * @brief Thread-safe timestamped logger for C++ server component.
 */

#pragma once

#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <ctime>

class Logger {
public:
    static void init(const std::string &path)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_.is_open()) file_.close();
        file_.open(path, std::ios::app);
        if (!file_.is_open())
            std::cerr << "[Logger] Cannot open: " << path << "\n";
    }

    static void info (const std::string &msg) { write("INFO ", msg); }
    static void warn (const std::string &msg) { write("WARN ", msg); }
    static void error(const std::string &msg) { write("ERROR", msg); }

    template<typename T>
    static void info (const std::string &msg, T val)
    { write("INFO ", fmt(msg, val)); }

    template<typename T>
    static void warn (const std::string &msg, T val)
    { write("WARN ", fmt(msg, val)); }

    template<typename T>
    static void error(const std::string &msg, T val)
    { write("ERROR", fmt(msg, val)); }

    static void close()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_.is_open()) file_.close();
    }

private:
    static std::ofstream file_;
    static std::mutex    mutex_;

    static std::string timestamp()
    {
        char buf[32];
        time_t now = time(nullptr);
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S",
                      std::localtime(&now));
        return buf;
    }

    static void write(const char *level, const std::string &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string line = "[" + timestamp() + "] [" +
                           level + "] " + msg + "\n";
        std::ostream &out = file_.is_open()
                          ? static_cast<std::ostream&>(file_)
                          : static_cast<std::ostream&>(std::cerr);
        out << line;
        out.flush();
        /* Also print to terminal */
        std::cout << line;
        std::cout.flush();
    }

    template<typename T>
    static std::string fmt(const std::string &tmpl, T val)
    {
        std::ostringstream oss;
        auto pos = tmpl.find("{}");
        if (pos != std::string::npos)
            oss << tmpl.substr(0, pos) << val << tmpl.substr(pos + 2);
        else
            oss << tmpl << " " << val;
        return oss.str();
    }
};

inline std::ofstream Logger::file_;
inline std::mutex    Logger::mutex_;

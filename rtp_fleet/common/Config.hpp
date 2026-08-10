/**
 * @file Config.hpp
 * @brief Simple KEY=VALUE config file parser for C++ components.
 */

#pragma once

#include <fstream>
#include <string>
#include <unordered_map>

class Config {
public:
    explicit Config(const std::string &path)
    {
        std::ifstream f(path);
        if (!f.is_open()) return;
        std::string line;
        while (std::getline(f, line)) {
            trim(line);
            if (line.empty() || line[0] == '#') continue;
            auto eq = line.find('=');
            if (eq == std::string::npos) continue;
            std::string key = line.substr(0, eq);
            std::string val = line.substr(eq + 1);
            trim(key); trim(val);
            map_[key] = val;
        }
    }

    std::string get(const std::string &key,
                    const std::string &def = "") const
    {
        auto it = map_.find(key);
        return (it != map_.end()) ? it->second : def;
    }

    int getInt(const std::string &key, int def = 0) const
    {
        auto it = map_.find(key);
        if (it == map_.end()) return def;
        try { return std::stoi(it->second); } catch (...) { return def; }
    }

    float getFloat(const std::string &key, float def = 0.0f) const
    {
        auto it = map_.find(key);
        if (it == map_.end()) return def;
        try { return std::stof(it->second); } catch (...) { return def; }
    }

private:
    std::unordered_map<std::string, std::string> map_;

    static void trim(std::string &s)
    {
        const char *ws = " \t\r\n";
        s.erase(0, s.find_first_not_of(ws));
        auto last = s.find_last_not_of(ws);
        if (last != std::string::npos) s.erase(last + 1);
        else s.clear();
    }
};

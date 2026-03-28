#pragma once

#include <deque>
#include <fstream>
#include <mutex>
#include <string>

#include <nlohmann/json.hpp>

using json = nlohmann::ordered_json;

class EventHistory {
public:
    EventHistory() = default;

    void init(size_t max_size) {
        max_size_ = max_size;
        file_path_ = "events.jsonl";
        loadFromDisk();
    }

    // Appends a raw JSON string to the ring buffer and persists it.
    // No JSON parsing — stored as-is, parsed only on getAll().
    void add(const std::string& json_payload) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (buffer_.size() >= max_size_) {
            buffer_.pop_front();
        }
        buffer_.push_back(json_payload);
        appendToDisk(json_payload);
    }

    // Returns all buffered events as a JSON array. Parses on read.
    json getAll() const {
        std::lock_guard<std::mutex> lk(mutex_);
        json arr = json::array();
        for (const auto& line : buffer_) {
            try {
                arr.push_back(json::parse(line));
            } catch (...) {
                // skip malformed lines
            }
        }
        return arr;
    }

private:
    void loadFromDisk() {
        std::ifstream f(file_path_);
        if (!f.is_open()) return;
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            if (buffer_.size() >= max_size_) buffer_.pop_front();
            buffer_.push_back(std::move(line));
        }
    }

    void appendToDisk(const std::string& json_payload) {
        std::ofstream f(file_path_, std::ios::app);
        if (f.is_open()) {
            f << json_payload << "\n";
        }
    }

    std::string file_path_;
    size_t max_size_ = 100;
    std::deque<std::string> buffer_;
    mutable std::mutex mutex_;
};

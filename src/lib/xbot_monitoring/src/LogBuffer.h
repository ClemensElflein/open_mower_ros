#pragma once

#include <cctype>
#include <deque>
#include <mutex>
#include <nlohmann/json.hpp>
#include <regex>
#include <string>

#include "rosgraph_msgs/Log.h"

using json = nlohmann::ordered_json;

// The last warnings and errors any node logged (/rosout_agg), kept in memory only, so apps can show what
// happened around a problem without reading the container's output.
class LogBuffer {
 public:
  explicit LogBuffer(size_t capacity) : capacity_(capacity) {}

  void add(const rosgraph_msgs::Log& log) {
    if (log.level < rosgraph_msgs::Log::WARN) return;
    std::lock_guard<std::mutex> lk(mutex_);
    entries_.push_back({log.header.stamp.toSec(), log.level, log.name, redact(log.msg)});
    if (entries_.size() > capacity_) entries_.pop_front();
  }

  // Oldest first. since: only entries after this unix time, min_level: WARN, ERROR or FATAL,
  // limit: at most that many of the newest ones.
  json get(double since, uint8_t min_level, size_t limit) const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<const Entry*> picked;
    for (auto it = entries_.rbegin(); it != entries_.rend() && picked.size() < limit; ++it) {
      if (it->t > since && it->level >= min_level) picked.push_back(&*it);
    }
    json result = json::array();
    for (auto it = picked.rbegin(); it != picked.rend(); ++it) {
      result.push_back({{"t", (*it)->t}, {"level", levelName((*it)->level)}, {"node", (*it)->node}, {"msg", (*it)->msg}});
    }
    return result;
  }

  static const char* levelName(uint8_t level) {
    switch (level) {
      case rosgraph_msgs::Log::WARN: return "WARN";
      case rosgraph_msgs::Log::ERROR: return "ERROR";
      case rosgraph_msgs::Log::FATAL: return "FATAL";
      default: return "INFO";
    }
  }

  // "WARN", "ERROR" or "FATAL" (any case), anything else counts as WARN
  static uint8_t levelFromName(std::string name) {
    for (auto& c : name) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (name == "ERROR") return rosgraph_msgs::Log::ERROR;
    if (name == "FATAL") return rosgraph_msgs::Log::FATAL;
    return rosgraph_msgs::Log::WARN;
  }

 private:
  struct Entry {
    double t;
    uint8_t level;
    std::string node;
    std::string msg;
  };

  // std::regex recurses per character, a line of some 50k characters overflows the stack and takes xbot_monitoring
  // down, so longer ones are cut first. At a character boundary: half an umlaut isn't valid UTF-8 and the json
  // answer couldn't be written anymore
  static constexpr size_t MAX_LENGTH = 2000;

  static std::string cut(const std::string& full) {
    if (full.size() <= MAX_LENGTH) return full;
    size_t end = MAX_LENGTH;
    // back to the first byte of a character that doesn't fit completely (continuation bytes are 10xxxxxx)
    while (end > 0 && (static_cast<unsigned char>(full[end]) & 0xC0) == 0x80) end--;
    std::string msg = full.substr(0, end);
    // a url that runs into the cut may have lost the @ ending its login, the password would show. so all of it goes
    const size_t url = msg.rfind("://");
    if (url != std::string::npos && msg.find_first_of(" \t\n\v\f\r", url) == std::string::npos) msg.erase(url + 3);
    return msg + "...";
  }

  // A log line may carry a login, e.g. an NTRIP or MQTT url with user and password, or a token in a header or in
  // json. Those are blanked.
  static std::string redact(const std::string& full) {
    const std::string msg = cut(full);
    // everything up to the last @ before the next space, a password may contain @ and / itself. a url with an @
    // in its path gets blanked a bit too much, but never too little
    static const std::regex url_login(R"((://)\S*@)");
    static const std::regex bearer(R"(\b(Bearer|Basic)\s+\S+)", std::regex::icase);
    // key = value, key: value, "key": "value"
    static const std::regex secret(
        R"(\b([a-z_]*(?:password|passwd|pwd|token|secret|api_?key|authorization))("?\s*[=:]\s*"?)[^\s",}]+)",
        std::regex::icase);
    std::string out = std::regex_replace(msg, url_login, "$1***@");
    out = std::regex_replace(out, bearer, "$1 ***");
    return std::regex_replace(out, secret, "$1$2***");
  }

  const size_t capacity_;
  std::deque<Entry> entries_;
  mutable std::mutex mutex_;
};

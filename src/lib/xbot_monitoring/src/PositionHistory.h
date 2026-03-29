#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <functional>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

using json = nlohmann::ordered_json;

class PositionHistory {
 public:
  PositionHistory() = default;

  void init(size_t max_segments, size_t simplify_threshold = 500) {
    file_path_ = "positions.jsonl";
    max_segments_ = max_segments;
    simplify_threshold_ = simplify_threshold;
    loadFromDisk();
    startNewSegment();
  }

  // Appends a raw point to the current open segment.
  // Closes segment if point count exceeds threshold to prevent unbounded memory usage.
  void addPoint(double x, double y, ros::Time t) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (current_.points.empty()) {
      current_.start_time = t;
    }
    current_.points.push_back({x, y});
    if (current_.points.size() >= simplify_threshold_) {
      startNewSegment();
    }
  }

  // Closes the current segment and opens a new one.
  // mow_enabled flips only on "MOWING_STARTED" / "MOWING_STOPPED".
  void onEvent(const std::string& type) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (type == "MOWING_STARTED") {
      current_mow_enabled_ = true;
    } else if (type == "MOWING_STOPPED") {
      current_mow_enabled_ = false;
    }
    startNewSegment();
  }

  // Simplifies and persists the current open segment.
  // Call on ROS shutdown to avoid losing in-progress track data.
  void flush() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (current_.points.empty()) return;
    startNewSegment();
  }

  // Returns all closed segments as a JSON array.
  json getHistory() const {
    std::lock_guard<std::mutex> lk(mutex_);
    json arr = json::array();
    for (const auto& seg : closed_segments_) {
      arr.push_back(segmentToJson(seg));
    }
    return arr;
  }

 private:
  struct Segment {
    ros::Time start_time;
    bool mow_enabled = false;
    std::vector<std::pair<double, double>> points;
  };

  void startNewSegment() {
    // Simplify and persist the current segment if it has points.
    if (!current_.points.empty()) {
      Segment simplified = current_;
      simplified.points = rdp(current_.points, 0.02);
      if (closed_segments_.size() >= max_segments_) {
        closed_segments_.pop_front();
      }
      closed_segments_.push_back(simplified);
      persistSegment(simplified);
    }

    // Start a new segment.
    current_ = Segment{};
    current_.mow_enabled = current_mow_enabled_;
  }

  void persistSegment(const Segment& seg) {
    std::string data = segmentToJson(seg).dump();
    std::lock_guard<std::mutex> lock(file_mutex_);
    std::ofstream f(file_path_, std::ios::app);
    if (f.is_open()) {
      f << data << "\n";
    }
  }

  void loadFromDisk() {
    std::ifstream f(file_path_);
    if (!f.is_open()) return;
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty()) continue;
      try {
        json j = json::parse(line);
        Segment seg;
        seg.start_time = ros::Time(j["start_time"].get<double>());
        seg.mow_enabled = j["mow_enabled"].get<bool>();
        for (const auto& pt : j["points"]) {
          seg.points.push_back({pt[0].get<double>(), pt[1].get<double>()});
        }
        if (closed_segments_.size() >= max_segments_) {
          closed_segments_.pop_front();
        }
        closed_segments_.push_back(seg);
      } catch (...) {
        // skip malformed lines
      }
    }
  }

  static json segmentToJson(const Segment& seg) {
    json pts = json::array();
    for (const auto& p : seg.points) {
      pts.push_back({p.first, p.second});
    }
    return {{"start_time", seg.start_time.toSec()}, {"mow_enabled", seg.mow_enabled}, {"points", pts}};
  }

  // Ramer-Douglas-Peucker line simplification.
  static double perpendicularDistance(const std::pair<double, double>& p, const std::pair<double, double>& a,
                                      const std::pair<double, double>& b) {
    double dx = b.first - a.first;
    double dy = b.second - a.second;
    double len2 = dx * dx + dy * dy;
    if (len2 == 0.0) {
      dx = p.first - a.first;
      dy = p.second - a.second;
      return std::sqrt(dx * dx + dy * dy);
    }
    double t = ((p.first - a.first) * dx + (p.second - a.second) * dy) / len2;
    t = std::max(0.0, std::min(1.0, t));
    double projx = a.first + t * dx;
    double projy = a.second + t * dy;
    dx = p.first - projx;
    dy = p.second - projy;
    return std::sqrt(dx * dx + dy * dy);
  }

  static void rdpRecursive(const std::vector<std::pair<double, double>>& pts, size_t start, size_t end, double epsilon,
                           std::vector<bool>& keep) {
    if (end <= start + 1) return;
    double max_dist = 0.0;
    size_t max_idx = start;
    for (size_t i = start + 1; i < end; ++i) {
      double d = perpendicularDistance(pts[i], pts[start], pts[end]);
      if (d > max_dist) {
        max_dist = d;
        max_idx = i;
      }
    }
    if (max_dist > epsilon) {
      keep[max_idx] = true;
      rdpRecursive(pts, start, max_idx, epsilon, keep);
      rdpRecursive(pts, max_idx, end, epsilon, keep);
    }
  }

  static std::vector<std::pair<double, double>> rdp(const std::vector<std::pair<double, double>>& pts, double epsilon) {
    if (pts.size() < 3) return pts;
    std::vector<bool> keep(pts.size(), false);
    keep.front() = true;
    keep.back() = true;
    rdpRecursive(pts, 0, pts.size() - 1, epsilon, keep);
    std::vector<std::pair<double, double>> result;
    result.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
      if (keep[i]) result.push_back(pts[i]);
    }
    return result;
  }

  std::string file_path_;
  size_t max_segments_ = 2000;
  size_t simplify_threshold_ = 500;
  bool current_mow_enabled_ = false;

  Segment current_;
  std::deque<Segment> closed_segments_;
  mutable std::mutex mutex_;
  mutable std::mutex file_mutex_;
};

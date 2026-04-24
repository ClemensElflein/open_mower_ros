#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using json = nlohmann::ordered_json;

/*
 * Two-pass position history pipeline, mirroring the OpenMower App.
 *
 * Input: median-filtered positions arriving every ~150 ms from pose_publish_timer_callback.
 *
 * The raw buffer is compacted into history segments when it exceeds COMPACTION_THRESHOLD:
 *   1. Decimation  — drops close/same-heading points, keeping a heartbeat.
 *   2. RDP         — further simplifies the decimated result.
 *
 * Segments carry a `blades` attribute. When it changes, a new segment is opened
 * and a bridge vertex (last point of previous segment) is prepended so segments connect.
 *
 * Persistence (append-only JSONL, two record types):
 *   {"type":"new_segment","blades":true,"points":[[x,y],...]}
 *   {"type":"append_points","points":[[x,y],...]}
 *
 * writePending() is called ONLY from periodicFlush() (timer) and flush() (shutdown).
 * No I/O happens on addPoint(), onEvent(), or any internal state change.
 *
 * Pending state: written_seg_count_ tracks how many segments have had their
 * new_segment record written; written_point_count_ tracks how many points of the
 * current tail segment have been written. Everything above those watermarks is pending.
 */

class PositionHistory {
 public:
  struct TrackAttributes {
    bool blades = false;
  };

  PositionHistory() = default;

  void init() {
    file_path_ = "positions.jsonl";
    loadFromDisk();
  }

  // Feed a median-filtered position. Called from the pose publish timer.
  void addPoint(double x, double y) {
    std::lock_guard<std::mutex> lk(mutex_);
    rel_buffer_.push_back({x, y});

    if (rel_buffer_.size() > COMPACTION_THRESHOLD) {
      compact(/*flush_all=*/false);
    }
  }

  // Interpret a parsed event JSON and update segment attributes accordingly.
  // Currently handles: MOWING_STARTED, MOWING_STOPPED.
  void onEvent(const json& event) {
    std::lock_guard<std::mutex> lk(mutex_);
    try {
      const std::string type = event.at("type").get<std::string>();
      if (type == "BLADES") {
        attributesChanged(current_attributes_.blades, event.at("enabled").get<bool>());
      }
    } catch (const json::exception&) {
      // malformed event — ignore
    }
  }

  // Write pending records to disk. Call from a 30s ROS timer.
  void periodicFlush() {
    std::lock_guard<std::mutex> lk(mutex_);
    writePending();
  }

  // Compact remaining buffer and write everything. Call on ROS shutdown.
  void flush() {
    std::lock_guard<std::mutex> lk(mutex_);
    compact(/*flush_all=*/true);
    writePending();
  }

  TrackAttributes getAttributes() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return current_attributes_;
  }

  // Returns all segments (including the open tail) as a JSON array for client seeding.
  // Format: [{"blades": bool, "points": [[x,y], ...]}, ...]
  json getHistory() const {
    std::lock_guard<std::mutex> lk(mutex_);
    json arr = json::array();
    for (const auto& seg : history_segments_) {
      arr.push_back(segmentToJson(seg));
    }
    return arr;
  }

 private:
  // --- Compaction ---
  static constexpr size_t COMPACTION_THRESHOLD = 60;
  static constexpr size_t CONTEXT_WINDOW = 10;

  // --- Decimation ---
  static constexpr double DIST_THRESHOLD = 0.1;         // 10 cm
  static constexpr double NOISE_DIST_THRESHOLD = 0.03;  // 3 cm
  static constexpr double HEADING_THRESHOLD_RAD = 8.0 * M_PI / 180.0;
  static constexpr size_t HEARTBEAT_POSITIONS = 10;
  // --- RDP ---
  static constexpr double RDP_EPSILON = 0.01;  // 1 cm

  struct Point {
    double x, y;
  };

  struct TaggedPoint {
    double x, y;
    bool committable;
  };

  struct Segment {
    std::vector<Point> points;
    TrackAttributes attributes;
  };

  // -------------------------------------------------------------------------
  // Decimation (mirrors decimate.ts)
  // -------------------------------------------------------------------------
  static double angleDiff(double a, double b) {
    double d = std::fmod(std::abs(a - b), 2.0 * M_PI);
    return d > M_PI ? 2.0 * M_PI - d : d;
  }

  static std::vector<TaggedPoint> decimateFilter(const std::vector<TaggedPoint>& pts) {
    static constexpr double DIST_THRESHOLD_SQ = DIST_THRESHOLD * DIST_THRESHOLD;
    static constexpr double NOISE_DIST_THRESH_SQ = NOISE_DIST_THRESHOLD * NOISE_DIST_THRESHOLD;

    if (pts.empty()) return {};
    std::vector<TaggedPoint> out = {pts[0]};
    double last_heading = std::numeric_limits<double>::quiet_NaN();
    size_t since_kept = 0;

    for (size_t i = 1; i < pts.size(); ++i) {
      const TaggedPoint& last = out.back();
      const TaggedPoint& curr = pts[i];
      ++since_kept;

      double dx = curr.x - last.x;
      double dy = curr.y - last.y;
      double dist_sq = dx * dx + dy * dy;

      // Only calculate and check heading if we moved enough to trust the vector
      double heading = last_heading;
      bool heading_changed = false;
      if (dist_sq > NOISE_DIST_THRESH_SQ) {
        heading = std::atan2(dy, dx);
        heading_changed = !std::isnan(last_heading) && angleDiff(last_heading, heading) > HEADING_THRESHOLD_RAD;
      }

      if (dist_sq > DIST_THRESHOLD_SQ || heading_changed || since_kept >= HEARTBEAT_POSITIONS) {
        out.push_back(pts[i]);
        last_heading = heading;
        since_kept = 0;
      }
    }
    return out;
  }

  // -------------------------------------------------------------------------
  // RDP (mirrors rdp.ts)
  // -------------------------------------------------------------------------
  static double pointToSegmentDistSq(const TaggedPoint& p, const TaggedPoint& a, const TaggedPoint& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double len2 = dx * dx + dy * dy;
    if (len2 == 0.0) {
      double ex = p.x - a.x, ey = p.y - a.y;
      return ex * ex + ey * ey;
    }
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
    t = std::max(0.0, std::min(1.0, t));
    double px = a.x + t * dx, py = a.y + t * dy;
    double ex = p.x - px, ey = p.y - py;
    return ex * ex + ey * ey;
  }

  static void rdpRecursive(const std::vector<TaggedPoint>& pts, size_t start, size_t end, std::vector<bool>& keep) {
    static constexpr double RDP_EPSILON_SQ = RDP_EPSILON * RDP_EPSILON;

    if (end <= start + 1) return;
    double max_dist_sq = 0.0;
    size_t max_idx = start;
    for (size_t i = start + 1; i < end; ++i) {
      double d_sq = pointToSegmentDistSq(pts[i], pts[start], pts[end]);
      if (d_sq > max_dist_sq) {
        max_dist_sq = d_sq;
        max_idx = i;
      }
    }
    if (max_dist_sq > RDP_EPSILON_SQ) {
      keep[max_idx] = true;
      rdpRecursive(pts, start, max_idx, keep);
      rdpRecursive(pts, max_idx, end, keep);
    }
  }

  static std::vector<TaggedPoint> rdpSimplify(const std::vector<TaggedPoint>& pts) {
    if (pts.size() < 3) return pts;
    std::vector<bool> keep(pts.size(), false);
    keep.front() = keep.back() = true;
    rdpRecursive(pts, 0, pts.size() - 1, keep);
    std::vector<TaggedPoint> result;
    result.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
      if (keep[i]) result.push_back(pts[i]);
    return result;
  }

  // -------------------------------------------------------------------------
  // Compaction (mirrors compactToHistory in useTrack.ts)
  // -------------------------------------------------------------------------
  void compact(bool flush_all) {
    if (rel_buffer_.size() <= CONTEXT_WINDOW && !flush_all) return;

    // Determine how many points to commit to the current segment.
    // Usually exclude the last CONTEXT_WINDOW points, unless flushing all.
    const size_t points_to_commit = flush_all ? rel_buffer_.size() : rel_buffer_.size() - CONTEXT_WINDOW;

    // Build the window for decimation and RDP. Include the last point of the previous segment as an anchor.
    std::vector<TaggedPoint> window;
    window.reserve(1 + rel_buffer_.size());  // at most 1 prefix point
    if (!history_segments_.empty() && !history_segments_.back().points.empty()) {
      const Point& anchor = history_segments_.back().points.back();
      window.push_back({anchor.x, anchor.y, /*committable=*/false});
    }
    for (size_t i = 0; i < rel_buffer_.size(); ++i) {
      window.push_back({rel_buffer_[i].x, rel_buffer_[i].y, /*committable=*/i < points_to_commit});
    }

    // Run decimation and RDP.
    std::vector<TaggedPoint> decimated = decimateFilter(window);
    std::vector<TaggedPoint> simplified = rdpSimplify(decimated);

    // Append committable points (excluding the anchor and last CONTEXT_WINDOW points) to history.
    std::vector<Point> committed;
    for (const TaggedPoint& p : simplified) {
      if (p.committable) committed.push_back({p.x, p.y});
    }
    if (!committed.empty()) {
      appendToHistory(committed);
    }

    rel_buffer_.erase(rel_buffer_.begin(), rel_buffer_.begin() + points_to_commit);
  }

  // Append committed points to the current open segment (or start one).
  void appendToHistory(const std::vector<Point>& pts) {
    if (history_segments_.empty()) {
      Segment seg;
      seg.attributes = current_attributes_;
      history_segments_.push_back(std::move(seg));
    }
    Segment& tail = history_segments_.back();
    tail.points.reserve(tail.points.size() + pts.size());
    tail.points.insert(tail.points.end(), pts.begin(), pts.end());
  }

  // -------------------------------------------------------------------------
  // Attribute changes
  // -------------------------------------------------------------------------

  void attributesChanged(bool& current, bool new_value) {
    if (current == new_value) return;
    compact(/*flush_all=*/true);
    current = new_value;
    startNewSegment();
  }

  // Close current open segment, bridge to the new one. Called when attributes change.
  void startNewSegment() {
    Segment seg;
    seg.attributes = current_attributes_;
    if (!history_segments_.empty() && !history_segments_.back().points.empty()) {
      // Prepend the last point of the previous segment as a bridge point.
      seg.points.push_back(history_segments_.back().points.back());
    }
    history_segments_.push_back(std::move(seg));
  }

  // -------------------------------------------------------------------------
  // Persistence
  // -------------------------------------------------------------------------
  //
  // Watermarks:
  //   written_seg_count_   — number of segments whose new_segment record has been written.
  //                          Segments [0, written_seg_count_) are fully on disk.
  //                          Segment written_seg_count_-1 may have more points pending
  //                          (tracked by written_point_count_).
  //   written_point_count_ — number of points of history_segments_[written_seg_count_-1]
  //                          that have already been written (inside the new_segment record
  //                          or subsequent append_points records).
  //
  // Called ONLY from periodicFlush() and flush() — never mid-operation.
  void writePending() {
    if (history_segments_.empty()) return;

    // Check if there is anything to write at all.
    // There is work if:
    //   a) any segment starting from written_seg_count_ has not had its record written, or
    //   b) the last written segment has grown beyond written_point_count_.
    const bool new_points =
        written_seg_count_ > 0 && history_segments_[written_seg_count_ - 1].points.size() > written_point_count_;
    const bool new_segments = written_seg_count_ < history_segments_.size();
    if (!new_points && !new_segments) return;

    std::ofstream f(file_path_, std::ios::app);
    if (!f.is_open()) {
      ROS_WARN_STREAM("PositionHistory: could not open " << file_path_ << " for writing");
      return;
    }

    // Append any new points that arrived in the current tail segment since the last write.
    if (new_points) {
      json j = {{"type", "append_points"}};
      json points = json::array();
      const Segment& tail = history_segments_[written_seg_count_ - 1];
      for (auto it = tail.points.begin() + written_point_count_; it != tail.points.end(); ++it) {
        points.push_back({it->x, it->y});
      }
      j["points"] = points;
      f << j.dump() << "\n";
      written_point_count_ = tail.points.size();
    }

    // Write new_segment records for every segment that hasn't been written yet.
    // Each carries all points accumulated so far in that segment (could be just a
    // bridge point, or bridge + some compacted points).
    for (auto it = history_segments_.begin() + written_seg_count_; it != history_segments_.end(); ++it) {
      json j = {{"type", "new_segment"}};
      j.update(segmentToJson(*it));
      f << j.dump() << "\n";
      ++written_seg_count_;
      written_point_count_ = it->points.size();
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
        std::string type = j.at("type").get<std::string>();

        if (type == "new_segment") {
          Segment seg;
          seg.attributes.blades = j.at("blades").get<bool>();
          for (const auto& pt : j.at("points")) {
            seg.points.push_back({pt[0].get<double>(), pt[1].get<double>()});
          }
          history_segments_.push_back(std::move(seg));
        } else if (type == "append_points") {
          if (history_segments_.empty()) continue;
          Segment& tail = history_segments_.back();
          for (const auto& pt : j.at("points")) {
            tail.points.push_back({pt[0].get<double>(), pt[1].get<double>()});
          }
        }
      } catch (const std::exception& e) {
        ROS_WARN_STREAM("PositionHistory: skipping malformed line: " << e.what());
      }
    }

    // All loaded segments are already on disk; set watermarks accordingly.
    // The last segment is "open" — we resume appending into it.
    // Restore current_attributes_ from the last segment so attribute-change
    // detection in onEvent() has the correct baseline.
    if (!history_segments_.empty()) {
      current_attributes_ = history_segments_.back().attributes;
      written_seg_count_ = history_segments_.size();
      written_point_count_ = history_segments_.back().points.size();
    }
  }

  static json segmentToJson(const Segment& seg) {
    json pts = json::array();
    for (const auto& p : seg.points) {
      pts.push_back({p.x, p.y});
    }
    return {{"blades", seg.attributes.blades}, {"points", pts}};
  }

  std::string file_path_;

  std::vector<Point> rel_buffer_;
  std::vector<Segment> history_segments_;

  // Current track attributes.
  TrackAttributes current_attributes_{.blades = false};

  // Persistence watermarks — monotonically advance, never go backward.
  // written_seg_count_:   segments [0, written_seg_count_) have their new_segment record on disk.
  // written_point_count_: points [0, written_point_count_) of segment [written_seg_count_-1] are on disk.
  size_t written_seg_count_ = 0;
  size_t written_point_count_ = 0;

  mutable std::mutex mutex_;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PositionHistory::TrackAttributes, blades)

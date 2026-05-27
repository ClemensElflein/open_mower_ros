#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
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
 * Points are only recorded while a mowing session is active.
 *
 * The raw buffer is compacted into history segments when it exceeds COMPACTION_THRESHOLD:
 *   1. Decimation  — drops close/same-heading points, keeping a heartbeat.
 *   2. RDP         — further simplifies the decimated result.
 *
 * Segments carry attributes (job_id, session_id, blades). When any attribute changes,
 * a new segment is opened and a bridge vertex (last point of previous segment) is
 * prepended so segments connect.
 *
 * Persistence (one append-only JSONL per job_id, stored in position_history/):
 *   Filename: <unix_epoch_seconds>_<job_id>.jsonl
 *   Two record types:
 *     {"type":"new_segment","attributes":{"job_id":"...","session_id":"...","blades":true},"points":[[x,y],...]}
 *     {"type":"append_points","points":[[x,y],...]}
 *
 * When job_id changes, the active file is closed (flushed) and a new one is opened.
 * session_id / blades changes only open a new segment inside the same file.
 *
 * Pending state: written_seg_count_ tracks how many segments have had their
 * new_segment record written; written_point_count_ tracks how many points of the
 * current tail segment have been written. Everything above those watermarks is pending.
 */

class PositionHistory {
 public:
  struct TrackAttributes {
    std::string job_id;
    std::string session_id;
    bool blades = false;
  };

  PositionHistory() = default;

  void init() {
    base_dir_ = "position_history";
    std::filesystem::create_directories(base_dir_);
    initializeFromDisk();
  }

  // Feed a median-filtered position. Called from the pose publish timer.
  // Points are only recorded when a session_id is present.
  void addPoint(double x, double y) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (current_attributes_.session_id.empty()) return;
    rel_buffer_.push_back({x, y});

    if (rel_buffer_.size() > COMPACTION_THRESHOLD) {
      compact(/*flush_all=*/false);
    }
  }

  // Interpret a parsed event JSON and update segment attributes accordingly.
  // Handles STATE (string job_id, string session_id) and BLADES (bool enabled).
  // A job_id change closes the current file and opens a new one.
  // Any other attribute change closes the current segment and opens a new one.
  void onEvent(const json& event) {
    std::lock_guard<std::mutex> lk(mutex_);
    try {
      const std::string type = event.at("type").get<std::string>();
      bool changed = false;
      bool job_changed = false;
      if (type == "STATE") {
        job_changed = updateAttribute(current_attributes_.job_id, event.value("job_id", std::string{}));
        changed =
            job_changed | updateAttribute(current_attributes_.session_id, event.value("session_id", std::string{}));
      } else if (type == "BLADES") {
        changed = updateAttribute(current_attributes_.blades, event.at("enabled").get<bool>());
      }
      if (job_changed) {
        compact(/*flush_all=*/true);
        writePending();
        resetState();
        if (!current_attributes_.job_id.empty()) {
          startNewSegment();
          writePending();
        }
      } else if (changed) {
        compact(/*flush_all=*/true);
        startNewSegment();
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

  // Returns compacted segments and the raw pending buffer for client seeding.
  // Always served from in-memory state (the active / latest job).
  // Format: {"segments": [...], "buffer": [[x,y],...]}
  json getHistory() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return buildHistoryJson(history_segments_, rel_buffer_);
  }

  // Returns the history for a specific job_id by loading its file from disk.
  // The active in-memory job is served from memory (no re-read).
  // Returns {"error": "not found"} if no file for that job_id exists.
  json getHistory(const std::string& job_id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (job_id == current_attributes_.job_id) {
      return buildHistoryJson(history_segments_, rel_buffer_);
    }
    for (const auto& e : dir_index_) {
      if (e.job_id == job_id) {
        auto segs = loadFile(e.path);
        std::vector<Point> empty_buf;
        return buildHistoryJson(segs, empty_buf);
      }
    }
    return {{"error", "not found"}};
  }

  // Returns array of {job_id, timestamp} sorted newest-first (derived from filenames).
  json listHistories() const {
    std::lock_guard<std::mutex> lk(mutex_);
    json arr = json::array();
    for (const auto& e : dir_index_) {
      arr.push_back({{"job_id", e.job_id}, {"timestamp", e.epoch}});
    }
    return arr;
  }

  // Deletes the file for job_id. If job_id is empty, deletes ALL files.
  // Cannot delete the currently active job's file.
  // Returns {"deleted": N}.
  json deleteHistory(const std::optional<std::string>& job_id) {
    std::lock_guard<std::mutex> lk(mutex_);
    int deleted = 0;
    for (auto it = dir_index_.begin(); it != dir_index_.end();) {
      if (job_id.has_value() && it->job_id != *job_id) {
        ++it;
        continue;
      }
      if (it->job_id == current_attributes_.job_id) {
        // Cannot delete the current file
        ++it;
        continue;
      }
      std::error_code ec;
      if (std::filesystem::remove(it->path, ec)) {
        ++deleted;
        it = dir_index_.erase(it);
      } else {
        ++it;
      }
    }
    return {{"deleted", deleted}};
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
    double started_at = 0.0;  // Unix time (seconds) of the first real compacted point
  };

  struct FileEntry {
    int64_t epoch;
    std::string job_id;
    std::filesystem::path path;
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
    if (tail.started_at == 0.0 && !pts.empty()) {
      tail.started_at = ros::Time::now().toSec();
    }
    tail.points.reserve(tail.points.size() + pts.size());
    tail.points.insert(tail.points.end(), pts.begin(), pts.end());
  }

  // -------------------------------------------------------------------------
  // Attribute changes
  // -------------------------------------------------------------------------

  template <typename T>
  bool updateAttribute(T& current, T new_value) {
    if (current == new_value) return false;
    current = std::move(new_value);
    return true;
  }

  // Close current open segment, bridge to the new one. Called when attributes change.
  void startNewSegment() {
    if (current_attributes_.job_id.empty()) return;
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

  // Reset in-memory state (called when job_id changes).
  void resetState() {
    history_segments_.clear();
    rel_buffer_.clear();
    written_seg_count_ = 0;
    written_point_count_ = 0;
    file_path_.clear();
  }

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

    if (!ensureFilePathSet()) return;

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

  // Ensure file_path_ is set for the current job.
  bool ensureFilePathSet() {
    if (!file_path_.empty()) return true;
    if (current_attributes_.job_id.empty()) return false;
    const int64_t epoch = static_cast<int64_t>(ros::Time::now().toSec());
    file_path_ = base_dir_ + "/" + std::to_string(epoch) + "_" + current_attributes_.job_id + ".jsonl";
    dir_index_.insert(dir_index_.begin(), {epoch, current_attributes_.job_id, file_path_});
    ROS_INFO_STREAM("PositionHistory: creating new file " << file_path_);
    return true;
  }

  // -------------------------------------------------------------------------
  // Directory scanning and file loading
  // -------------------------------------------------------------------------

  // Parse filenames of the form "<epoch>_<job_id>.jsonl".
  // Returns entries sorted newest-first (descending epoch).
  std::vector<FileEntry> scanDir() const {
    std::vector<FileEntry> entries;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(base_dir_, ec)) {
      if (!entry.is_regular_file()) continue;
      const std::string name = entry.path().filename().string();
      if (name.size() < 7 || name.substr(name.size() - 6) != ".jsonl") continue;
      const std::string stem = name.substr(0, name.size() - 6);
      const auto sep = stem.find('_');
      if (sep == std::string::npos || sep == 0) continue;
      try {
        int64_t epoch = std::stoll(stem.substr(0, sep));
        std::string job_id = stem.substr(sep + 1);
        if (job_id.empty()) continue;
        entries.push_back({epoch, job_id, entry.path()});
      } catch (...) {
        continue;
      }
    }
    std::sort(entries.begin(), entries.end(), [](const FileEntry& a, const FileEntry& b) { return a.epoch > b.epoch; });
    return entries;
  }

  static std::vector<Segment> loadFile(const std::filesystem::path& path) {
    std::vector<Segment> segs;
    std::ifstream f(path);
    if (!f.is_open()) return segs;

    std::string line;
    while (std::getline(f, line)) {
      if (line.empty()) continue;
      try {
        json j = json::parse(line);
        std::string type = j.at("type").get<std::string>();

        if (type == "new_segment") {
          Segment seg;
          const auto& attrs = j.at("attributes");
          seg.attributes.job_id = attrs.value("job_id", std::string{});
          seg.attributes.session_id = attrs.value("session_id", std::string{});
          seg.attributes.blades = attrs.at("blades").get<bool>();
          seg.started_at = j.value("started_at", 0.0);
          for (const auto& pt : j.at("points")) {
            seg.points.push_back({pt[0].get<double>(), pt[1].get<double>()});
          }
          segs.push_back(std::move(seg));
        } else if (type == "append_points") {
          if (segs.empty()) continue;
          Segment& tail = segs.back();
          for (const auto& pt : j.at("points")) {
            tail.points.push_back({pt[0].get<double>(), pt[1].get<double>()});
          }
        }
      } catch (const std::exception& e) {
        ROS_WARN_STREAM("PositionHistory: skipping malformed line in " << path << ": " << e.what());
      }
    }
    return segs;
  }

  // Load the latest file (by epoch) into in-memory state on startup.
  // Also populates dir_index_.
  void initializeFromDisk() {
    dir_index_ = scanDir();
    if (dir_index_.empty()) return;

    const FileEntry& latest = dir_index_.front();
    history_segments_ = loadFile(latest.path);

    if (!history_segments_.empty()) {
      // Restore only job_id so file rotation detects changes correctly.
      // session_id and blades must come from fresh STATE/BLADES events after boot;
      // restoring them would cause addPoint() to record while the mower is docked.
      current_attributes_.job_id = history_segments_.back().attributes.job_id;
      file_path_ = latest.path.string();
      ROS_INFO_STREAM("PositionHistory: loaded existing file " << file_path_);

      // Open a fresh segment (no bridge point) so post-boot points start a clean track.
      Segment seg;
      seg.attributes = current_attributes_;
      history_segments_.push_back(std::move(seg));

      written_seg_count_ = history_segments_.size() - 1;
      written_point_count_ = 0;
    }
  }

  static json buildHistoryJson(const std::vector<Segment>& segs, const std::vector<Point>& buf) {
    json j_segs = json::array();
    for (const auto& seg : segs) {
      j_segs.push_back(segmentToJson(seg));
    }
    json j_buf = json::array();
    for (const auto& p : buf) {
      j_buf.push_back({p.x, p.y});
    }
    return {{"segments", j_segs}, {"buffer", j_buf}};
  }

  static json segmentToJson(const Segment& seg) {
    json pts = json::array();
    for (const auto& p : seg.points) {
      pts.push_back({p.x, p.y});
    }
    return {{"attributes",
             {{"job_id", seg.attributes.job_id},
              {"session_id", seg.attributes.session_id},
              {"blades", seg.attributes.blades}}},
            {"started_at", seg.started_at},
            {"points", pts}};
  }

  std::string base_dir_;
  std::string file_path_;
  std::vector<FileEntry> dir_index_;  // populated once at startup, kept in sync

  std::vector<Point> rel_buffer_;
  std::vector<Segment> history_segments_;

  // Current track attributes.
  TrackAttributes current_attributes_{};

  // Persistence watermarks — monotonically advance, never go backward.
  // written_seg_count_:   segments [0, written_seg_count_) have their new_segment record on disk.
  // written_point_count_: points [0, written_point_count_) of segment [written_seg_count_-1] are on disk.
  size_t written_seg_count_ = 0;
  size_t written_point_count_ = 0;

  mutable std::mutex mutex_;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PositionHistory::TrackAttributes, job_id, session_id, blades)

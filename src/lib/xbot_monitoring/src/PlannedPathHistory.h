#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using json = nlohmann::ordered_json;

/*
 * Per-job planned-path history, mirroring PositionHistory's persistence model for the actual track.
 *
 * mower_logic publishes the slic3r plan once per step as {job_id, step_index, area_id, angle, offset,
 * paths:[...]}. step_index is the plan's ordinal position in the job (the sole storage key); today
 * it is the area index (areas are mowed once each, in order), and the planned Tasklist feature must
 * keep sending a per-job pass ordinal so the same area mowed more than once stays a distinct step.
 * area_id/angle/offset are metadata only. A job_id change starts a new job.
 *
 * To keep MQTT payloads bounded regardless of lawn size, nothing carries the whole job at once:
 *   - the live overlay publishes only the CURRENT area (getCurrentArea);
 *   - history is fetched one area at a time -- getSteps lists a job's steps (metadata only) and
 *     getStep returns one area's geometry.
 *
 * Persistence: one directory per job, one independently-written file per step:
 *   planned_path/<epoch>_<job_id>/area_<step_index>.json = {"area_id","angle","offset","paths":[...]}
 * A new or re-sliced step only rewrites its own small file. The job_id and creation timestamp come
 * from the directory name (same convention as PositionHistory's filenames), so no metadata file is
 * needed.
 */
class PlannedPathHistory {
 public:
  void init() {
    base_dir_ = "planned_path";
    std::filesystem::create_directories(base_dir_);
    dir_index_ = scanDir();
    // Load the latest job into memory so the live overlay can be re-published after a restart mid-job.
    if (!dir_index_.empty() && loadJob(dir_index_.front().path, areas_)) {
      current_job_id_ = dir_index_.front().job_id;
      job_dir_ = dir_index_.front().path.string();
    }
  }

  // Feed one area/task's published plan. Accumulates into the current job, keyed by step_index (the
  // plan's ordinal position in the job); a job_id change starts a new job directory.
  void addPlan(const json& msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!msg.is_object() || !msg.contains("job_id") || !msg["job_id"].is_string()) return;
    const std::string job_id = msg["job_id"].get<std::string>();
    if (job_id.empty()) return;

    AreaPlan plan;
    plan.area_id = msg.value("area_id", std::string{});
    plan.angle = msg.value("angle", 0.0);
    plan.offset = msg.value("offset", 0.0);
    plan.paths = msg.contains("paths") ? msg["paths"] : json::array();

    if (job_id != current_job_id_) {
      current_job_id_ = job_id;
      areas_.clear();
      const int64_t epoch = epochNow();
      job_dir_ = base_dir_ + "/" + std::to_string(epoch) + "_" + job_id;
      std::error_code ec;
      std::filesystem::create_directories(job_dir_, ec);
      dir_index_.insert(dir_index_.begin(), {epoch, job_id, job_dir_});
    }

    // Key purely by step_index: the plan's ordinal position in the job (area order today, task order
    // with the planned Tasklist). It is unique per pass and stable across a resume re-slice, so the
    // same area mowed twice -- even at the same angle -- is two distinct steps, while re-slicing a
    // step overwrites it in place. No content (area_id/angle) is used as a key, since any of it can
    // legitimately repeat. A missing/invalid index falls back to appending.
    const int step =
        (msg.contains("step_index") && msg["step_index"].is_number_integer()) ? msg["step_index"].get<int>() : -1;
    size_t idx = (step >= 0 && step <= kMaxAreaIndex) ? static_cast<size_t>(step) : areas_.size();
    if (idx > kMaxAreaIndex) return;
    if (idx >= areas_.size()) areas_.resize(idx + 1);
    areas_[idx] = std::move(plan);
    current_step_idx_ = static_cast<int>(idx);  // the area just (re)published is the live one
    saveArea(idx);
    live_seen_ = true;
  }

  // Whether there is a LIVE plan to publish as the overlay. init() loads the latest job for history,
  // but we don't resurrect it as the live overlay until mower_logic actually (re)publishes it this
  // session, so a finished job isn't shown as live after a restart (mirrors PositionHistory, which
  // restores a job for history without re-growing its track).
  bool hasCurrent() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return live_seen_ && current_step_idx_ >= 0 && static_cast<size_t>(current_step_idx_) < areas_.size();
  }

  // The current area being mowed, for the live overlay: {"job_id","step_index","paths":[...]}.
  // Only the current area is published live (not the whole job) so the MQTT payload stays bounded;
  // the full job is reachable one area at a time via the history RPCs.
  json getCurrentArea() const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (current_step_idx_ < 0 || static_cast<size_t>(current_step_idx_) >= areas_.size())
      return {{"job_id", current_job_id_}, {"step_index", -1}, {"paths", json::array()}};
    const json& p = areas_[static_cast<size_t>(current_step_idx_)].paths;
    return {{"job_id", current_job_id_}, {"step_index", current_step_idx_}, {"paths", p.is_array() ? p : json::array()}};
  }

  // Lists a job's steps as metadata only (no geometry), so the app can fetch each area separately:
  // {"job_id","steps":[{"step_index","area_id","angle","offset"}]}. {"error":"not found"} for an
  // unknown job_id; with no job_id, the current/latest job.
  json getSteps(const std::string& job_id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!current_job_id_.empty() && job_id == current_job_id_) return stepsOf(current_job_id_, areas_);
    for (const auto& e : dir_index_) {
      if (e.job_id != job_id) continue;
      std::vector<AreaPlan> areas;
      if (loadJob(e.path, areas)) return stepsOf(job_id, areas);
      break;
    }
    return {{"error", "not found"}};
  }

  json getSteps() const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!current_job_id_.empty()) return stepsOf(current_job_id_, areas_);
    if (!dir_index_.empty()) {
      std::vector<AreaPlan> areas;
      if (loadJob(dir_index_.front().path, areas)) return stepsOf(dir_index_.front().job_id, areas);
    }
    return {{"job_id", ""}, {"steps", json::array()}};
  }

  // One step's geometry: {"job_id","step_index","paths":[...]} -- the current job from memory, a past
  // job from its single area_<step_index>.json file. {"error":"not found"} if absent.
  json getStep(const std::string& job_id, int step_index) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (step_index < 0) return {{"error", "not found"}};
    if (!current_job_id_.empty() && job_id == current_job_id_) {
      const size_t i = static_cast<size_t>(step_index);
      if (i < areas_.size() && areas_[i].paths.is_array() && !areas_[i].paths.empty())
        return {{"job_id", job_id}, {"step_index", step_index}, {"paths", areas_[i].paths}};
      return {{"error", "not found"}};
    }
    for (const auto& e : dir_index_) {
      if (e.job_id != job_id) continue;
      json paths;
      if (loadArea(e.path, step_index, paths)) return {{"job_id", job_id}, {"step_index", step_index}, {"paths", paths}};
      break;
    }
    return {{"error", "not found"}};
  }

  // Array of {job_id, timestamp} sorted newest-first.
  json listHistories() const {
    std::lock_guard<std::mutex> lk(mutex_);
    json arr = json::array();
    for (const auto& e : dir_index_) arr.push_back({{"job_id", e.job_id}, {"timestamp", e.epoch}});
    return arr;
  }

  // Deletes the directory for job_id (or ALL jobs when job_id is empty). Never deletes the active job.
  json deleteHistory(const std::optional<std::string>& job_id) {
    std::lock_guard<std::mutex> lk(mutex_);
    int deleted = 0;
    for (auto it = dir_index_.begin(); it != dir_index_.end();) {
      if (job_id.has_value() && it->job_id != *job_id) {
        ++it;
        continue;
      }
      if (it->job_id == current_job_id_) {
        ++it;
        continue;
      }
      std::error_code ec;
      std::filesystem::remove_all(it->path, ec);
      if (!ec) {
        ++deleted;
        it = dir_index_.erase(it);
      } else {
        ++it;
      }
    }
    return {{"deleted", deleted}};
  }

 private:
  struct AreaPlan {
    std::string area_id;  // metadata only (which map area) -- never a key
    double angle = 0;     // metadata: slic3r mowing angle used for this pass (radians)
    double offset = 0;    // metadata: mow-angle offset applied for this pass
    json paths;
  };
  struct JobEntry {
    int64_t epoch;
    std::string job_id;
    std::filesystem::path path;
  };

  // Upper bound on a per-area file index (guards the load-time resize against a corrupt filename).
  static constexpr int kMaxAreaIndex = 10000;

  static int64_t epochNow() {
    return static_cast<int64_t>(ros::Time::now().toSec());
  }

  // Build the metadata-only step list for a job: {"job_id","steps":[{step_index,area_id,angle,offset}]}.
  // Skips empty placeholder slots (gaps left by a missing area file).
  static json stepsOf(const std::string& job_id, const std::vector<AreaPlan>& areas) {
    json steps = json::array();
    for (size_t i = 0; i < areas.size(); i++) {
      const AreaPlan& a = areas[i];
      if (!a.paths.is_array() || a.paths.empty()) continue;
      steps.push_back({{"step_index", static_cast<int>(i)},
                       {"area_id", a.area_id},
                       {"angle", a.angle},
                       {"offset", a.offset}});
    }
    return {{"job_id", job_id}, {"steps", steps}};
  }

  // Read a single area_<step_index>.json file's paths from a job dir. False if missing/empty.
  static bool loadArea(const std::filesystem::path& dir, int step_index, json& paths) {
    std::ifstream f(dir / ("area_" + std::to_string(step_index) + ".json"));
    if (!f.is_open()) return false;
    try {
      json doc;
      f >> doc;
      if (!doc.contains("paths") || !doc["paths"].is_array() || doc["paths"].empty()) return false;
      paths = doc["paths"];
      return true;
    } catch (const json::exception&) {
      return false;
    }
  }

  static bool writeFileAtomic(const std::string& path, const std::string& data) {
    const std::string tmp = path + ".tmp";
    {
      std::ofstream os(tmp, std::ios::trunc);
      if (!os) {
        ROS_WARN_STREAM("PlannedPathHistory: cannot open '" << tmp << "' for writing");
        return false;
      }
      os << data;
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
      ROS_WARN_STREAM("PlannedPathHistory: cannot finalize '" << path << "': " << ec.message());
      return false;
    }
    return true;
  }

  // Write a single area's file (only this area is rewritten when it changes).
  void saveArea(size_t idx) const {
    if (job_dir_.empty() || idx >= areas_.size()) return;
    json doc;
    doc["area_id"] = areas_[idx].area_id;
    doc["angle"] = areas_[idx].angle;
    doc["offset"] = areas_[idx].offset;
    doc["paths"] = areas_[idx].paths;
    writeFileAtomic(job_dir_ + "/area_" + std::to_string(idx) + ".json", doc.dump());
  }

  // Parse "area_<i>.json" into its index. Returns false if the name doesn't match or the index is
  // implausibly large (a guard against a crafted/corrupt filename blowing up the load-time resize).
  static bool parseAreaIndex(const std::string& name, int& out) {
    if (name.rfind("area_", 0) != 0 || name.size() < 11 || name.compare(name.size() - 5, 5, ".json") != 0) return false;
    const std::string num = name.substr(5, name.size() - 10);
    if (num.empty()) return false;
    for (char c : num)
      if (c < '0' || c > '9') return false;
    try {
      out = std::stoi(num);
      return out >= 0 && out <= kMaxAreaIndex;
    } catch (...) {
      return false;
    }
  }

  // Load a job directory's area_<i>.json files into `areas`, each placed at its own index.
  static bool loadJob(const std::filesystem::path& dir, std::vector<AreaPlan>& areas) {
    std::vector<std::pair<int, std::filesystem::path>> files;
    std::error_code ec;
    int max_idx = -1;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
      int idx = 0;
      if (e.is_regular_file() && parseAreaIndex(e.path().filename().string(), idx)) {
        files.emplace_back(idx, e.path());
        max_idx = std::max(max_idx, idx);
      }
    }
    if (files.empty()) return false;

    // Place each area at its own index so the in-memory index always matches the file index, even
    // if a middle area_<i>.json is missing; a gap stays an empty placeholder (skipped by the step
    // list) rather than shifting later areas, which would otherwise let a later re-slice write a duplicate.
    areas.assign(static_cast<size_t>(max_idx) + 1, AreaPlan{});
    for (const auto& f : files) {
      json doc;
      std::ifstream af(f.second);
      if (af.is_open()) {
        try {
          af >> doc;
        } catch (const json::exception&) {
          doc = json::object();
        }
      }
      areas[static_cast<size_t>(f.first)] = {doc.value("area_id", std::string{}), doc.value("angle", 0.0),
                                             doc.value("offset", 0.0),
                                             doc.contains("paths") ? doc["paths"] : json::array()};
    }
    return true;
  }

  // Scan planned_path/ for "<epoch>_<job_id>" job directories, newest-first by epoch.
  std::vector<JobEntry> scanDir() const {
    std::vector<JobEntry> entries;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(base_dir_, ec)) {
      if (!entry.is_directory()) continue;
      const std::string name = entry.path().filename().string();
      const auto sep = name.find('_');
      if (sep == std::string::npos || sep == 0) continue;
      try {
        const int64_t epoch = std::stoll(name.substr(0, sep));
        const std::string job_id = name.substr(sep + 1);
        if (!job_id.empty()) entries.push_back({epoch, job_id, entry.path()});
      } catch (...) {
        continue;
      }
    }
    std::sort(entries.begin(), entries.end(), [](const JobEntry& a, const JobEntry& b) { return a.epoch > b.epoch; });
    return entries;
  }

  std::string base_dir_;
  std::string job_dir_;
  std::vector<JobEntry> dir_index_;
  std::string current_job_id_;
  std::vector<AreaPlan> areas_;
  int current_step_idx_ = -1;  // index of the area most recently (re)published = the live overlay area
  bool live_seen_ = false;     // a plan was (re)published this session → safe to show as the live overlay
  mutable std::mutex mutex_;
};

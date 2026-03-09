/*
 * Copyright 2026 Zeke Sarosi.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Terrain data server plugin
 * @file terrain_server.cpp
 * @author Zeke Sarosi
 *
 * @addtogroup plugin
 * @{
 *
 * Implements the GCS/companion-computer side of the MAVLink Terrain Protocol.
 * Reference: https://mavlink.io/en/services/terrain.html
 *
 * Published topics (relative to plugin namespace):
 * - ~/report  (mavros_msgs/TerrainReport): terrain elevation responses
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <list>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <curl/curl.h>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/terrain_report.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include "srtm_continent_map.hpp"

extern char ** environ;   // POSIX.1 — needed for posix_spawnp (unzip)

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Terrain data server plugin.
 * @plugin terrain_server
 *
 * Serves SRTM terrain altitude data to the FCU in response to
 * TERRAIN_REQUEST messages.  Loads .hgt files from a configurable
 * directory, with optional automatic downloading from a terrain
 * server.  Drip-feeds TERRAIN_DATA responses at a configurable rate
 * to avoid flooding the MAVLink link.
 *
 * Published topics (relative to plugin namespace):
 * - ~/report  (mavros_msgs/TerrainReport): terrain elevation responses
 */
class TerrainServerPlugin : public plugin::Plugin
{
public:
  explicit TerrainServerPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "terrain_server")
  {
    node->declare_parameter("terrain_data_path", std::string(""));
    node->declare_parameter("send_rate_hz", 5.0);
    node->declare_parameter("auto_download", false);
    node->declare_parameter("download_host", std::string("terrain.ardupilot.org"));
    node->declare_parameter("srtm_source", std::string("SRTM3"));
    node->declare_parameter("offline", false);
    node->declare_parameter("max_cache_tiles", 64);

    node->get_parameter("terrain_data_path", terrain_data_path_);
    node->get_parameter("auto_download", auto_download_);
    node->get_parameter("download_host", download_host_);
    node->get_parameter("srtm_source", srtm_source_);
    node->get_parameter("offline", offline_);
    node->get_parameter("max_cache_tiles", max_cache_tiles_);

    double rate_hz = 5.0;
    node->get_parameter("send_rate_hz", rate_hz);
    if (rate_hz <= 0.0) {rate_hz = 5.0;}

    if (terrain_data_path_.empty() && auto_download_) {
      const char * home = std::getenv("HOME");
      terrain_data_path_ = home
        ? std::string(home) + "/.cache/mavros/terrain/" + srtm_source_
        : "/tmp/mavros_terrain/" + srtm_source_;
      RCLCPP_INFO(get_logger(),
        "Auto-download enabled — cache directory: %s",
        terrain_data_path_.c_str());
    }

    if (!terrain_data_path_.empty()) {
      try {
        std::filesystem::create_directories(terrain_data_path_);
      } catch (const std::filesystem::filesystem_error & e) {
        RCLCPP_ERROR(get_logger(),
          "Cannot create terrain directory: %s", e.what());
      }
      build_file_index(terrain_data_path_);
    } else {
      RCLCPP_WARN(get_logger(),
        "terrain_data_path not set and auto_download disabled — "
        "plugin will not serve terrain");
    }

    if (auto_download_ && !offline_) {
      for (int i = 0; i < MAX_DOWNLOAD_WORKERS; ++i) {
        download_workers_.emplace_back(
          &TerrainServerPlugin::download_worker_loop, this);
      }
    }

    terrain_report_pub_ =
      node->create_publisher<mavros_msgs::msg::TerrainReport>("~/report", 10);

    send_timer_ = node->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&TerrainServerPlugin::send_next_tile, this));

    uas->diagnostic_updater.add(
      "Terrain Server", this, &TerrainServerPlugin::diag_run);
  }

  ~TerrainServerPlugin() override
  {
    shutting_down_ = true;

    uas->diagnostic_updater.removeByName("Terrain Server");

    if (send_timer_) {
      send_timer_->cancel();
    }

    download_cv_.notify_all();
    for (auto & w : download_workers_) {
      if (w.joinable()) {w.join();}
    }
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&TerrainServerPlugin::handle_terrain_request),
      make_handler(&TerrainServerPlugin::handle_terrain_check),
    };
  }

private:
  // --------------------------------------------------------- constants
  static constexpr int SRTM1_SIDE = 3601;
  static constexpr int SRTM3_SIDE = 1201;
  static constexpr int16_t SRTM_VOID = -32768;
  static constexpr int GRID_COLS = 8;
  static constexpr int GRID_ROWS = 7;
  static constexpr int TILE_DIM = 4;
  static constexpr double RADIUS_OF_EARTH = 6378100.0;
  static constexpr int MAX_DOWNLOAD_WORKERS = 2;
  static constexpr size_t MAX_FAILED_ENTRIES = 512;

  // --------------------------------------------------------- types
  struct SrtmTile {
    std::vector<int16_t> data;
    int side = 0;
  };

  struct LruEntry {
    int64_t key;
    SrtmTile tile;
  };

  struct DownloadJob {
    int lat;
    int lon;
  };

  struct PendingRequest {
    int32_t lat;
    int32_t lon;
    uint16_t grid_spacing;
    uint64_t mask;
    uint64_t sent_mask = 0;
  };

  // --------------------------------------------------------- LRU tile cache
  std::list<LruEntry> lru_list_;
  std::unordered_map<int64_t, std::list<LruEntry>::iterator> lru_index_;
  std::unordered_map<std::string, std::filesystem::path> file_index_;
  std::mutex cache_mutex_;

  // --------------------------------------------------------- download state
  std::queue<DownloadJob> download_queue_;
  std::unordered_set<int64_t> download_enqueued_;
  std::unordered_set<int64_t> download_failed_;
  std::mutex download_mutex_;
  std::condition_variable download_cv_;
  std::vector<std::thread> download_workers_;

  // --------------------------------------------------------- request queue
  std::vector<PendingRequest> pending_requests_;
  std::mutex req_mutex_;

  // --------------------------------------------------------- diagnostics
  std::atomic<uint64_t> requests_received_{0};
  std::atomic<uint64_t> blocks_sent_{0};

  // --------------------------------------------------------- config
  std::string terrain_data_path_;
  std::string download_host_;
  std::string srtm_source_;
  bool auto_download_ = false;
  bool offline_ = false;
  int max_cache_tiles_ = 64;
  std::atomic<bool> shutting_down_{false};

  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::Publisher<mavros_msgs::msg::TerrainReport>::SharedPtr
    terrain_report_pub_;

  // ====================================================== static helpers

  static int64_t tile_key(int lat, int lon)
  {
    return static_cast<int64_t>(lat + 90) * 360 + (lon + 180);
  }

  static void tile_filename(int lat, int lon, char * buf, size_t len)
  {
    char ns = lat >= 0 ? 'N' : 'S';
    char ew = lon >= 0 ? 'E' : 'W';
    std::snprintf(buf, len, "%c%02d%c%03d.hgt",
      ns, std::abs(lat), ew, std::abs(lon));
  }

  // ----------------------------------------------------- geodesic math
  //  Direct port of MAVProxy mp_util.gps_newpos / gps_offset (rhumb line).

  static std::pair<double, double> gps_newpos(
    double lat_deg, double lon_deg, double bearing_deg, double distance_m)
  {
    if (distance_m == 0.0) {
      return {lat_deg, lon_deg};
    }

    double lat1 = std::clamp(
      lat_deg * M_PI / 180.0, -M_PI / 2 + 1e-15, M_PI / 2 - 1e-15);
    double lon1 = lon_deg * M_PI / 180.0;
    double tc = -bearing_deg * M_PI / 180.0;
    double d = distance_m / RADIUS_OF_EARTH;

    double lat = lat1 + d * std::cos(tc);
    lat = std::clamp(lat, -M_PI / 2 + 1e-15, M_PI / 2 - 1e-15);

    double q;
    if (std::abs(lat - lat1) < 1e-15) {
      q = std::cos(lat1);
    } else {
      double dphi = std::log(
        std::tan(lat / 2 + M_PI / 4) / std::tan(lat1 / 2 + M_PI / 4));
      q = (lat - lat1) / dphi;
    }

    double dlon = -d * std::sin(tc) / q;
    double lon = std::fmod(lon1 + dlon + M_PI, 2 * M_PI) - M_PI;

    return {lat * 180.0 / M_PI, lon * 180.0 / M_PI};
  }

  static std::pair<double, double> gps_offset(
    double lat_deg, double lon_deg, double east_m, double north_m)
  {
    double bearing = std::atan2(east_m, north_m) * 180.0 / M_PI;
    double distance = std::sqrt(east_m * east_m + north_m * north_m);
    return gps_newpos(lat_deg, lon_deg, bearing, distance);
  }

  // ----------------------------------------------------- HTTP download

  static size_t curl_write_cb(void * ptr, size_t size, size_t nmemb, void * ud)
  {
    auto * fp = static_cast<std::FILE *>(ud);
    return std::fwrite(ptr, size, nmemb, fp);
  }

  bool download_to_file(
    const std::string & url, const std::string & path,
    long connect_timeout_s = 10, long max_time_s = 60)
  {
    CURL * curl = curl_easy_init();
    if (!curl) {
      RCLCPP_ERROR(get_logger(), "curl_easy_init failed");
      return false;
    }

    std::FILE * fp = std::fopen(path.c_str(), "wb");
    if (!fp) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s for writing", path.c_str());
      curl_easy_cleanup(curl);
      return false;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, connect_timeout_s);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, max_time_s);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    CURLcode res = curl_easy_perform(curl);

    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_easy_cleanup(curl);
    std::fclose(fp);

    if (res != CURLE_OK) {
      RCLCPP_WARN(get_logger(),
        "Download failed: %s (HTTP %ld) — %s",
        url.c_str(), http_code, curl_easy_strerror(res));
      std::filesystem::remove(path);
      return false;
    }

    return true;
  }

  // ----------------------------------------------------- process execution
  //  Kept only for zip extraction (unzip / python3 fallback).

  static int run_process(const std::vector<std::string> & args)
  {
    std::vector<char *> argv;
    argv.reserve(args.size() + 1);
    for (const auto & a : args) {
      argv.push_back(const_cast<char *>(a.c_str()));
    }
    argv.push_back(nullptr);

    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(
      &actions, STDOUT_FILENO, "/dev/null", O_WRONLY, 0);
    posix_spawn_file_actions_addopen(
      &actions, STDERR_FILENO, "/dev/null", O_WRONLY, 0);

    pid_t pid;
    int rc = posix_spawnp(
      &pid, argv[0], &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);

    if (rc != 0) {return -1;}

    int status;
    if (waitpid(pid, &status, 0) < 0) {return -1;}
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  }

  // ====================================================== file index

  void build_file_index(const std::string & path)
  {
    try {
      if (!std::filesystem::exists(path)) {
        RCLCPP_ERROR(get_logger(),
          "Terrain data path does not exist: %s", path.c_str());
        return;
      }

      int count = 0;
      for (const auto & entry :
        std::filesystem::recursive_directory_iterator(path))
      {
        if (entry.path().extension() == ".hgt") {
          file_index_[entry.path().filename().string()] = entry.path();
          ++count;
        }
      }

      RCLCPP_INFO(get_logger(),
        "Terrain directory indexed — %d .hgt files in %s",
        count, path.c_str());
    } catch (const std::filesystem::filesystem_error & e) {
      RCLCPP_ERROR(get_logger(),
        "Error indexing terrain directory: %s", e.what());
    }
  }

  // ====================================================== SRTM loading

  SrtmTile load_srtm_tile(int lat, int lon)
  {
    SrtmTile tile;
    if (terrain_data_path_.empty()) {return tile;}

    try {
      char filename[32];
      tile_filename(lat, lon, filename, sizeof(filename));

      std::filesystem::path filepath;
      {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        auto it = file_index_.find(filename);
        if (it != file_index_.end()) {
          filepath = it->second;
        }
      }

      if (filepath.empty()) {
        filepath = std::filesystem::path(terrain_data_path_) / filename;
        if (!std::filesystem::exists(filepath)) {return tile;}
        std::lock_guard<std::mutex> lock(cache_mutex_);
        file_index_[filename] = filepath;
      }

      auto file_size = std::filesystem::file_size(filepath);
      size_t expected_srtm1 =
        static_cast<size_t>(SRTM1_SIDE) * SRTM1_SIDE * 2;
      size_t expected_srtm3 =
        static_cast<size_t>(SRTM3_SIDE) * SRTM3_SIDE * 2;

      if (file_size == expected_srtm1) {
        tile.side = SRTM1_SIDE;
      } else if (file_size == expected_srtm3) {
        tile.side = SRTM3_SIDE;
      } else {
        RCLCPP_WARN(get_logger(),
          "Unexpected file size for %s: %ju bytes",
          filename, static_cast<uintmax_t>(file_size));
        return tile;
      }

      std::ifstream file(filepath, std::ios::binary);
      if (!file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Cannot open: %s", filepath.c_str());
        tile.side = 0;
        return tile;
      }

      size_t n = static_cast<size_t>(tile.side) * tile.side;
      tile.data.resize(n);
      file.read(
        reinterpret_cast<char *>(tile.data.data()),
        static_cast<std::streamsize>(n * sizeof(int16_t)));

      if (!file.good() ||
        static_cast<size_t>(file.gcount()) != n * sizeof(int16_t))
      {
        RCLCPP_ERROR(get_logger(),
          "Short read on %s (got %zd of %zu bytes)",
          filename,
          static_cast<size_t>(file.gcount()),
          n * sizeof(int16_t));
        tile.side = 0;
        tile.data.clear();
        return tile;
      }

      for (auto & val : tile.data) {
        val = static_cast<int16_t>(ntohs(static_cast<uint16_t>(val)));
      }

      RCLCPP_INFO(get_logger(), "Loaded SRTM tile %s (%d x %d)",
        filename, tile.side, tile.side);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(),
        "Exception loading SRTM tile: %s", e.what());
      tile.side = 0;
      tile.data.clear();
    }

    return tile;
  }

  // ====================================================== LRU cache

  void evict_lru()
  {
    while (max_cache_tiles_ > 0 &&
      static_cast<int>(lru_index_.size()) > max_cache_tiles_)
    {
      if (lru_list_.empty()) {break;}
      lru_index_.erase(lru_list_.back().key);
      lru_list_.pop_back();
    }
  }

  SrtmTile * get_tile(int lat, int lon)
  {
    int64_t key = tile_key(lat, lon);
    bool needs_download = false;

    // Phase 1: fast cache check
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      auto it = lru_index_.find(key);
      if (it != lru_index_.end()) {
        lru_list_.splice(lru_list_.begin(), lru_list_, it->second);
        return it->second->tile.side > 0 ? &it->second->tile : nullptr;
      }
    }

    // Phase 2: load from disk (no lock — avoids blocking other lookups)
    auto tile = load_srtm_tile(lat, lon);

    // Phase 3: insert into cache
    SrtmTile * result = nullptr;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      auto it = lru_index_.find(key);
      if (it != lru_index_.end()) {
        lru_list_.splice(lru_list_.begin(), lru_list_, it->second);
        result = it->second->tile.side > 0 ? &it->second->tile : nullptr;
      } else {
        bool loaded = tile.side > 0;
        evict_lru();
        lru_list_.push_front({key, std::move(tile)});
        lru_index_[key] = lru_list_.begin();
        if (loaded) {
          result = &lru_list_.front().tile;
        } else if (auto_download_ && !offline_) {
          needs_download = true;
        }
      }
    }

    // Phase 4: enqueue download outside any lock (prevents ordering issues)
    if (needs_download) {
      enqueue_download(lat, lon);
    }

    return result;
  }

  void invalidate_cached_miss(int64_t key)
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = lru_index_.find(key);
    if (it != lru_index_.end() && it->second->tile.side == 0) {
      lru_list_.erase(it->second);
      lru_index_.erase(it);
    }
  }

  // ====================================================== download management

  void enqueue_download(int lat, int lon)
  {
    int64_t key = tile_key(lat, lon);
    std::lock_guard<std::mutex> lock(download_mutex_);

    if (download_enqueued_.count(key) || download_failed_.count(key)) {
      return;
    }

    download_enqueued_.insert(key);
    download_queue_.push({lat, lon});
    download_cv_.notify_one();
  }

  void download_worker_loop()
  {
    while (true) {
      DownloadJob job{};
      {
        std::unique_lock<std::mutex> lock(download_mutex_);
        download_cv_.wait(lock, [this]() {
            return shutting_down_.load() || !download_queue_.empty();
          });
        if (shutting_down_) {return;}
        job = download_queue_.front();
        download_queue_.pop();
      }

      try {
        download_tile(job.lat, job.lon);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(),
          "Exception in terrain download: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(),
          "Unknown exception in terrain download");
      }

      int64_t key = tile_key(job.lat, job.lon);
      {
        std::lock_guard<std::mutex> lock(download_mutex_);
        download_enqueued_.erase(key);
      }

      invalidate_cached_miss(key);
    }
  }

  void download_tile(int lat, int lon)
  {
    if (shutting_down_) {return;}

    char filename[32];
    tile_filename(lat, lon, filename, sizeof(filename));

    std::string hgt_path =
      (std::filesystem::path(terrain_data_path_) / filename).string();

    if (std::filesystem::exists(hgt_path)) {return;}

    auto continent = srtm::lookup_continent(lat, lon);
    if (!continent) {
      RCLCPP_DEBUG(get_logger(),
        "Tile %s outside SRTM coverage — likely ocean", filename);
      mark_download_failed(lat, lon);
      return;
    }

    std::string zip_name = std::string(filename) + ".zip";
    std::string zip_path =
      (std::filesystem::path(terrain_data_path_) / zip_name).string();

    std::string url = "https://" + download_host_ + "/" +
      srtm_source_ + "/" + *continent + zip_name;

    RCLCPP_INFO(get_logger(), "Downloading %s", url.c_str());

    if (!download_to_file(url, zip_path)) {
      mark_download_failed(lat, lon);
      return;
    }

    if (shutting_down_) {
      std::filesystem::remove(zip_path);
      return;
    }

    int rc = run_process(
      {"unzip", "-o", "-q", zip_path, "-d", terrain_data_path_});

    if (rc != 0) {
      rc = run_process({
          "python3", "-c",
          "import zipfile,sys; "
          "zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])",
          zip_path, terrain_data_path_});
    }

    std::filesystem::remove(zip_path);

    if (rc != 0) {
      RCLCPP_ERROR(get_logger(),
        "Failed to extract %s — install 'unzip' or python3",
        zip_name.c_str());
      mark_download_failed(lat, lon);
      return;
    }

    if (std::filesystem::exists(hgt_path)) {
      RCLCPP_INFO(get_logger(), "Downloaded and cached: %s", filename);
      std::lock_guard<std::mutex> lock(cache_mutex_);
      file_index_[filename] = hgt_path;
    } else {
      RCLCPP_WARN(get_logger(),
        "Extraction produced no .hgt file for %s", filename);
      mark_download_failed(lat, lon);
    }
  }

  void mark_download_failed(int lat, int lon)
  {
    std::lock_guard<std::mutex> lock(download_mutex_);
    if (download_failed_.size() >= MAX_FAILED_ENTRIES) {
      download_failed_.erase(download_failed_.begin());
    }
    download_failed_.insert(tile_key(lat, lon));
  }

  // ====================================================== elevation lookup

  double lookup_elevation(double lat_deg, double lon_deg)
  {
    int tlat = static_cast<int>(std::floor(lat_deg));
    int tlon = static_cast<int>(std::floor(lon_deg));

    auto * t = get_tile(tlat, tlon);
    if (!t) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    double frac_lat = lat_deg - tlat;
    double frac_lon = lon_deg - tlon;

    double row_d = (1.0 - frac_lat) * (t->side - 1);
    double col_d = frac_lon * (t->side - 1);

    int r0 = std::clamp(
      static_cast<int>(std::floor(row_d)), 0, t->side - 2);
    int c0 = std::clamp(
      static_cast<int>(std::floor(col_d)), 0, t->side - 2);

    double fr = row_d - r0;
    double fc = col_d - c0;

    auto sample = [&](int r, int c) -> double {
        int16_t v = t->data[r * t->side + c];
        return v == SRTM_VOID
          ? std::numeric_limits<double>::quiet_NaN()
          : static_cast<double>(v);
      };

    double v00 = sample(r0, c0);
    double v01 = sample(r0, c0 + 1);
    double v10 = sample(r0 + 1, c0);
    double v11 = sample(r0 + 1, c0 + 1);

    if (std::isnan(v00) || std::isnan(v01) ||
      std::isnan(v10) || std::isnan(v11))
    {
      for (double v : {v00, v01, v10, v11}) {
        if (!std::isnan(v)) {return v;}
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    return (v00 * (1 - fc) + v01 * fc) * (1 - fr) +
           (v10 * (1 - fc) + v11 * fc) * fr;
  }

  // ====================================================== MAVLink RX

  void handle_terrain_request(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::TERRAIN_REQUEST & request,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    std::lock_guard<std::mutex> lock(req_mutex_);

    for (auto & req : pending_requests_) {
      if (req.lat == request.lat && req.lon == request.lon &&
        req.grid_spacing == request.grid_spacing)
      {
        req.mask = request.mask;
        req.sent_mask &= request.mask;
        ++requests_received_;
        return;
      }
    }

    pending_requests_.push_back({
        request.lat, request.lon, request.grid_spacing,
        request.mask, 0});
    ++requests_received_;

    RCLCPP_INFO(get_logger(),
      "TERRAIN_REQUEST lat=%.7f lon=%.7f spacing=%u "
      "mask=0x%016lx (queue=%zu)",
      request.lat / 1e7, request.lon / 1e7,
      request.grid_spacing,
      static_cast<unsigned long>(request.mask),
      pending_requests_.size());
  }

  void handle_terrain_check(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::TERRAIN_CHECK & check,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    double lat = check.lat / 1e7;
    double lon = check.lon / 1e7;

    double elev = lookup_elevation(lat, lon);
    if (std::isnan(elev)) {
      RCLCPP_WARN(get_logger(),
        "TERRAIN_CHECK: no data for lat=%.7f lon=%.7f", lat, lon);
      return;
    }

    mavlink::common::msg::TERRAIN_REPORT rpt{};
    rpt.lat = check.lat;
    rpt.lon = check.lon;
    rpt.spacing = 0;
    rpt.terrain_height = static_cast<float>(elev);
    rpt.current_height = 0;
    rpt.pending = 0;
    rpt.loaded = 1;

    uas->send_message(rpt);

    auto ros_msg = mavros_msgs::msg::TerrainReport();
    ros_msg.header.stamp = node->now();
    ros_msg.header.frame_id = "terrain";
    ros_msg.latitude = lat;
    ros_msg.longitude = lon;
    ros_msg.spacing = 0;
    ros_msg.terrain_height = static_cast<float>(elev);
    ros_msg.current_height = 0;
    ros_msg.pending = 0;
    ros_msg.loaded = 1;
    terrain_report_pub_->publish(ros_msg);
  }

  // ====================================================== timer-driven TX

  void send_next_tile()
  {
    PendingRequest req{};
    size_t req_idx = 0;
    bool found = false;

    {
      std::lock_guard<std::mutex> lock(req_mutex_);

      pending_requests_.erase(
        std::remove_if(
          pending_requests_.begin(), pending_requests_.end(),
          [](const PendingRequest & r) {
            return (r.mask & ~r.sent_mask) == 0;
          }),
        pending_requests_.end());

      for (size_t i = 0; i < pending_requests_.size(); ++i) {
        if ((pending_requests_[i].mask &
          ~pending_requests_[i].sent_mask) != 0)
        {
          req = pending_requests_[i];
          req_idx = i;
          found = true;
          break;
        }
      }
    }

    if (!found) {return;}

    uint64_t needed = req.mask & ~req.sent_mask;
    for (int bit = 0; bit < GRID_COLS * GRID_ROWS; ++bit) {
      if (!(needed & (1ULL << bit))) {continue;}

      if (send_terrain_data_bit(req, bit)) {
        std::lock_guard<std::mutex> lock(req_mutex_);
        if (req_idx < pending_requests_.size() &&
          pending_requests_[req_idx].lat == req.lat &&
          pending_requests_[req_idx].lon == req.lon)
        {
          pending_requests_[req_idx].sent_mask |= (1ULL << bit);
          ++blocks_sent_;

          auto & r = pending_requests_[req_idx];
          if ((r.mask & ~r.sent_mask) == 0) {
            RCLCPP_INFO(get_logger(),
              "All terrain tiles served for lat=%.7f lon=%.7f",
              r.lat / 1e7, r.lon / 1e7);
            pending_requests_.erase(
              pending_requests_.begin() +
              static_cast<std::ptrdiff_t>(req_idx));
          }
        }
      }
      return;
    }
  }

  bool send_terrain_data_bit(const PendingRequest & req, int bit)
  {
    double base_lat = req.lat / 1e7;
    double base_lon = req.lon / 1e7;
    double spacing = static_cast<double>(req.grid_spacing);
    double bit_spacing = spacing * TILE_DIM;

    int col = bit % GRID_COLS;
    int row = bit / GRID_COLS;

    auto [tile_lat, tile_lon] = gps_offset(
      base_lat, base_lon,
      bit_spacing * col,
      bit_spacing * row);

    int16_t data[16];

    for (int i = 0; i < 16; ++i) {
      int y = i % TILE_DIM;
      int x = i / TILE_DIM;

      auto [pt_lat, pt_lon] = gps_offset(
        tile_lat, tile_lon,
        spacing * y,
        spacing * x);

      double elev = lookup_elevation(pt_lat, pt_lon);
      if (std::isnan(elev)) {
        RCLCPP_DEBUG(get_logger(),
          "No elevation at (%.7f, %.7f) for gridbit %d",
          pt_lat, pt_lon, bit);
        return false;
      }
      data[i] = static_cast<int16_t>(std::round(elev));
    }

    mavlink::common::msg::TERRAIN_DATA td{};
    td.lat = req.lat;
    td.lon = req.lon;
    td.grid_spacing = req.grid_spacing;
    td.gridbit = static_cast<uint8_t>(bit);
    std::copy(std::begin(data), std::end(data), std::begin(td.data));

    uas->send_message(td);
    return true;
  }

  // ====================================================== diagnostics

  void diag_run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    int cached = 0;
    int loaded = 0;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      cached = static_cast<int>(lru_index_.size());
      for (const auto & entry : lru_list_) {
        if (entry.tile.side > 0) {++loaded;}
      }
    }

    int pending_dl = 0;
    int failed = 0;
    {
      std::lock_guard<std::mutex> lock(download_mutex_);
      pending_dl = static_cast<int>(download_enqueued_.size());
      failed = static_cast<int>(download_failed_.size());
    }

    size_t req_count;
    {
      std::lock_guard<std::mutex> lock(req_mutex_);
      req_count = pending_requests_.size();
    }

    if (terrain_data_path_.empty() && !auto_download_) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, "Not configured");
    } else if (pending_dl > 0) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK, "Downloading");
    } else if (req_count > 0) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK, "Serving");
    } else {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK, "Idle");
    }

    stat.addf("Requests received", "%lu",
      static_cast<unsigned long>(requests_received_.load()));
    stat.addf("Blocks sent", "%lu",
      static_cast<unsigned long>(blocks_sent_.load()));
    stat.addf("Pending FCU requests", "%zu", req_count);
    stat.addf("Tiles loaded", "%d", loaded);
    stat.addf("Tiles cached (incl. misses)", "%d", cached);
    stat.addf("Downloads pending", "%d", pending_dl);
    stat.addf("Downloads failed", "%d", failed);
    stat.add("Auto download", auto_download_ ? "yes" : "no");
    stat.add("Data path",
      terrain_data_path_.empty() ? "(none)" : terrain_data_path_);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::TerrainServerPlugin)

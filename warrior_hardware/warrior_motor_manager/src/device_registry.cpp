#include "warrior_motor_manager/device_registry.hpp"

#include <glob.h>

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include "warrior_motor_manager/serial_protocol.hpp"

namespace warrior::hardware {

namespace {

constexpr auto ARDUINO_RESET_DELAY     = std::chrono::milliseconds(2000);
constexpr auto HANDSHAKE_TIMEOUT       = std::chrono::milliseconds(3000);
constexpr auto SPARK_DISCOVER_TIMEOUT  = std::chrono::milliseconds(3000);

std::vector<std::string> list_arduino_candidate_ports()
{
    std::vector<std::string> ports;
    for (const char * pattern : {"/dev/ttyACM*", "/dev/ttyUSB*"}) {
        glob_t g{};
        if (glob(pattern, 0, nullptr, &g) == 0) {
            for (std::size_t i = 0; i < g.gl_pathc; ++i) {
                ports.emplace_back(g.gl_pathv[i]);
            }
        }
        globfree(&g);
    }
    std::sort(ports.begin(), ports.end());
    return ports;
}

}  // namespace

DeviceRegistry::DeviceRegistry(rclcpp::Logger logger,
                               ArduinoConfig arduino_cfg,
                               SparkConfig spark_cfg,
                               std::chrono::duration<double> discovery_period)
: logger_(std::move(logger))
, arduino_cfg_(std::move(arduino_cfg))
, spark_cfg_(std::move(spark_cfg))
, discovery_period_(discovery_period)
{
}

DeviceRegistry::~DeviceRegistry()
{
    stop();
}

void DeviceRegistry::start()
{
    if (discovery_thread_.joinable()) return;
    stop_flag_ = false;
    discovery_thread_ = std::thread(&DeviceRegistry::discovery_loop, this);
}

void DeviceRegistry::stop()
{
    {
        std::lock_guard<std::mutex> lock(stop_mutex_);
        stop_flag_ = true;
    }
    stop_cv_.notify_all();
    if (discovery_thread_.joinable()) {
        discovery_thread_.join();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    arduinos_.clear();
    sparks_.clear();
    last_spark_status_.clear();
}

// ── Arduino API ──────────────────────────────────────────────────────────

bool DeviceRegistry::is_arduino_connected(const std::string & name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return arduinos_.find(name) != arduinos_.end();
}

std::vector<std::string> DeviceRegistry::connected_arduino_names()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> names;
    names.reserve(arduinos_.size());
    for (const auto & [name, _] : arduinos_) names.push_back(name);
    std::sort(names.begin(), names.end());
    return names;
}

bool DeviceRegistry::send_drive_percent(const std::string & name, int percent)
{
    const std::string frame = serial_protocol::encode_drive(name, percent);

    std::lock_guard<std::mutex> lock(mutex_);
    auto it = arduinos_.find(name);
    if (it == arduinos_.end()) return false;
    if (!it->second->write_frame(frame)) {
        RCLCPP_WARN(logger_, "[%s] write failed; dropping connection on %s",
            name.c_str(), it->second->port().c_str());
        arduinos_.erase(it);
        return false;
    }
    return true;
}

std::vector<std::pair<std::string, std::string>> DeviceRegistry::drain_arduino_messages()
{
    std::vector<std::pair<std::string, std::string>> out;
    std::vector<std::string> to_drop;

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & [name, dev] : arduinos_) {
        auto lines = dev->read_pending_lines();
        if (!dev->is_open()) {
            to_drop.push_back(name);
            continue;
        }
        for (auto & line : lines) {
            out.emplace_back(name, std::move(line));
        }
    }
    for (const auto & name : to_drop) {
        arduinos_.erase(name);
    }
    return out;
}

void DeviceRegistry::send_zero_to_all_arduinos()
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & [name, dev] : arduinos_) {
        dev->write_frame(serial_protocol::encode_drive(name, 0));
    }
}

// ── SPARK MAX API ────────────────────────────────────────────────────────

bool DeviceRegistry::is_spark_connected(int can_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    return it != sparks_.end() && it->second && it->second->is_open();
}

std::vector<int> DeviceRegistry::connected_spark_can_ids()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<int> out;
    out.reserve(sparks_.size());
    for (const auto & [id, sess] : sparks_) {
        if (sess && sess->is_open()) out.push_back(id);
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::string DeviceRegistry::spark_port(int can_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    if (it == sparks_.end() || !it->second) return {};
    return it->second->port();
}

bool DeviceRegistry::send_steer_position(int can_id, float motor_rotations)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    if (it == sparks_.end() || !it->second || !it->second->is_open()) return false;
    it->second->set_target_position(motor_rotations);
    it->second->enable();
    return true;
}

void DeviceRegistry::release_steer(int can_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    if (it == sparks_.end() || !it->second) return;
    it->second->disable();
}

std::optional<float> DeviceRegistry::spark_position_rot(int can_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    if (it == sparks_.end() || !it->second) return std::nullopt;
    const float p = it->second->position_rotations();
    if (std::isnan(p)) return std::nullopt;
    return p;
}

std::optional<float> DeviceRegistry::spark_applied_pct(int can_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    if (it == sparks_.end() || !it->second) return std::nullopt;
    return it->second->applied_output_percent();
}

double DeviceRegistry::seconds_since_spark_position(int can_id, rclcpp::Time now)
{
    // Approximate: we track "last Status 2 count seen" per discovery sweep
    // in last_spark_status_; if status_2_count is increasing, refresh the
    // timestamp. This avoids hanging a third thread off each session.
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    if (it == sparks_.end() || !it->second) return -1.0;

    static thread_local std::unordered_map<int, uint64_t> last_count;
    const uint64_t cnt = it->second->status_2_count();
    if (cnt > last_count[can_id]) {
        last_count[can_id] = cnt;
        last_spark_status_[can_id] = now;
    }
    auto ts_it = last_spark_status_.find(can_id);
    if (ts_it == last_spark_status_.end()) return -1.0;
    return (now - ts_it->second).seconds();
}

// ── Discovery ────────────────────────────────────────────────────────────

std::vector<std::string> DeviceRegistry::missing_arduinos_locked() const
{
    std::vector<std::string> missing;
    for (const auto & name : arduino_cfg_.wanted_names) {
        if (arduinos_.find(name) == arduinos_.end()) {
            missing.push_back(name);
        }
    }
    return missing;
}

std::vector<int> DeviceRegistry::missing_sparks_locked() const
{
    std::vector<int> missing;
    for (int id : spark_cfg_.wanted_can_ids) {
        if (sparks_.find(id) == sparks_.end()) {
            missing.push_back(id);
        }
    }
    return missing;
}

std::vector<std::string> DeviceRegistry::snapshot_claimed_ports_locked() const
{
    std::vector<std::string> ports;
    ports.reserve(arduinos_.size() + sparks_.size());
    for (const auto & [_, dev] : arduinos_) ports.push_back(dev->port());
    for (const auto & [_, sess] : sparks_) {
        if (sess && sess->is_open()) ports.push_back(sess->port());
    }
    return ports;
}

bool DeviceRegistry::wait_arduino_reset()
{
    std::unique_lock<std::mutex> lock(stop_mutex_);
    return stop_cv_.wait_for(lock, ARDUINO_RESET_DELAY,
                             [this] { return stop_flag_.load(); });
}

std::optional<std::string> DeviceRegistry::probe_arduino(const std::string & port,
                                                          ArduinoSerialDevice & dev)
{
    if (!dev.open(port, arduino_cfg_.baud_rate)) return std::nullopt;
    if (wait_arduino_reset()) {  // true if stop requested
        dev.close();
        return std::nullopt;
    }
    return dev.handshake(HANDSHAKE_TIMEOUT);
}

std::unique_ptr<SparkMaxSession> DeviceRegistry::probe_spark(const std::string & port)
{
    auto sess = std::make_unique<SparkMaxSession>();
    if (!sess->open(port)) return nullptr;

    const auto deadline = std::chrono::steady_clock::now() + SPARK_DISCOVER_TIMEOUT;
    while (std::chrono::steady_clock::now() < deadline) {
        if (stop_flag_) { sess->close(); return nullptr; }
        if (sess->device_id() >= 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (sess->device_id() < 0) {
        // Heartbeat blasted for 3 s with no inbound traffic — controller is
        // silent (cold boot? Status streams disabled? 12 V off?). Drop.
        sess->close();
        return nullptr;
    }
    return sess;
}

void DeviceRegistry::scan_once()
{
    std::vector<std::string> missing_arduinos;
    std::vector<int>         missing_sparks;
    std::unordered_set<std::string> claimed_ports;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        missing_arduinos = missing_arduinos_locked();
        missing_sparks   = missing_sparks_locked();
        for (const auto & p : snapshot_claimed_ports_locked()) {
            claimed_ports.insert(p);
        }
    }

    if (missing_arduinos.empty() && missing_sparks.empty()) return;

    RCLCPP_INFO(logger_, "[discovery] scanning: %zu Arduino(s) missing, %zu SPARK(s) missing",
        missing_arduinos.size(), missing_sparks.size());

    // ── Phase 1: SPARK MAXes (faster — no DTR reset wait) ─────────────────
    if (!missing_sparks.empty()) {
        const auto spark_ports = list_sparkmax_ports();
        for (const auto & port : spark_ports) {
            if (stop_flag_) return;
            if (claimed_ports.count(port)) continue;
            if (missing_sparks.empty()) break;

            auto sess = probe_spark(port);
            if (!sess) continue;
            const int dev = sess->device_id();

            auto want_it = std::find(missing_sparks.begin(), missing_sparks.end(), dev);
            if (want_it == missing_sparks.end()) {
                RCLCPP_DEBUG(logger_, "[discovery] SPARK dev=%d on %s — not wanted, releasing",
                    dev, port.c_str());
                sess->close();
                continue;
            }
            RCLCPP_INFO(logger_, "[discovery] connected SPARK dev=%d on %s",
                dev, port.c_str());
            {
                std::lock_guard<std::mutex> lock(mutex_);
                sparks_[dev] = std::move(sess);
            }
            missing_sparks.erase(want_it);
            claimed_ports.insert(port);
        }
    }

    // ── Phase 2: Arduinos ────────────────────────────────────────────────
    if (!missing_arduinos.empty()) {
        const auto candidate_ports = list_arduino_candidate_ports();
        for (const auto & port : candidate_ports) {
            if (stop_flag_) return;
            if (claimed_ports.count(port)) continue;
            if (missing_arduinos.empty()) break;

            auto dev = std::make_unique<ArduinoSerialDevice>();
            const auto name_opt = probe_arduino(port, *dev);
            if (!name_opt) continue;
            const std::string & name = *name_opt;

            auto wanted_it = std::find(missing_arduinos.begin(), missing_arduinos.end(), name);
            if (wanted_it == missing_arduinos.end()) {
                RCLCPP_DEBUG(logger_, "[discovery] Arduino %s on %s — not wanted, releasing",
                    name.c_str(), port.c_str());
                continue;
            }
            RCLCPP_INFO(logger_, "[discovery] connected Arduino %s on %s",
                name.c_str(), port.c_str());
            {
                std::lock_guard<std::mutex> lock(mutex_);
                arduinos_[name] = std::move(dev);
            }
            missing_arduinos.erase(wanted_it);
            claimed_ports.insert(port);
        }
    }
}

void DeviceRegistry::discovery_loop()
{
    while (!stop_flag_) {
        scan_once();
        std::unique_lock<std::mutex> lock(stop_mutex_);
        stop_cv_.wait_for(lock, discovery_period_, [this] { return stop_flag_.load(); });
    }
}

}  // namespace warrior::hardware

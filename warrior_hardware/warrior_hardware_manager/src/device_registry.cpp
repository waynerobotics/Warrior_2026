#include "warrior_hardware/device_registry.hpp"

#include <glob.h>

#include <algorithm>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

#include "warrior_hardware/serial_protocol.hpp"

namespace warrior::hardware {

namespace {

constexpr auto ARDUINO_RESET_DELAY = std::chrono::milliseconds(2000);
constexpr auto HANDSHAKE_TIMEOUT   = std::chrono::milliseconds(3000);

std::vector<std::string> list_candidate_ports()
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
                               SlcanConfig slcan_cfg,
                               std::chrono::duration<double> discovery_period)
: logger_(std::move(logger))
, arduino_cfg_(std::move(arduino_cfg))
, slcan_cfg_(std::move(slcan_cfg))
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
    slcan_.reset();
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

// ── SLCAN API ────────────────────────────────────────────────────────────

bool DeviceRegistry::is_slcan_connected()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return slcan_ && slcan_->is_open();
}

std::string DeviceRegistry::slcan_port()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (slcan_ && slcan_->is_open()) return slcan_->port();
    return {};
}

bool DeviceRegistry::send_steer_position(int can_id, float motor_rotations)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!slcan_ || !slcan_->is_open()) return false;
    if (!slcan_->send_position(can_id, motor_rotations)) {
        RCLCPP_WARN(logger_, "[slcan] write failed on %s; dropping",
            slcan_->port().c_str());
        slcan_.reset();
        return false;
    }
    return true;
}

std::vector<sparkmax::CanFrame> DeviceRegistry::drain_slcan_frames()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!slcan_ || !slcan_->is_open()) return {};
    auto frames = slcan_->read_pending_frames();
    if (!slcan_->is_open()) {
        // read_pending_frames() closed it on POSIX error — drop.
        slcan_.reset();
    }
    return frames;
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

std::vector<std::string> DeviceRegistry::snapshot_claimed_ports_locked() const
{
    std::vector<std::string> ports;
    ports.reserve(arduinos_.size() + 1);
    for (const auto & [_, dev] : arduinos_) ports.push_back(dev->port());
    if (slcan_ && slcan_->is_open()) ports.push_back(slcan_->port());
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
    if (wait_arduino_reset()) {  // returns true if stop requested
        dev.close();
        return std::nullopt;
    }
    return dev.handshake(HANDSHAKE_TIMEOUT);
}

void DeviceRegistry::scan_once()
{
    // Snapshot what we want and what we already own
    std::vector<std::string> missing_arduinos;
    std::unordered_set<std::string> claimed_ports;
    bool slcan_missing = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        missing_arduinos = missing_arduinos_locked();
        for (const auto & p : snapshot_claimed_ports_locked()) {
            claimed_ports.insert(p);
        }
        slcan_missing = !(slcan_ && slcan_->is_open());
    }

    if (missing_arduinos.empty() && !slcan_missing) return;

    RCLCPP_INFO(logger_, "[discovery] scanning: %zu Arduino(s) missing, slcan=%s",
        missing_arduinos.size(), slcan_missing ? "missing" : "ok");

    const auto candidate_ports = list_candidate_ports();

    // ── Phase 1: explicit SLCAN port (if configured) ──────────────────────
    if (slcan_missing && slcan_cfg_.port != "auto" && !slcan_cfg_.port.empty()) {
        const std::string & port = slcan_cfg_.port;
        if (claimed_ports.count(port) == 0 &&
            std::find(candidate_ports.begin(), candidate_ports.end(), port) != candidate_ports.end())
        {
            auto dev = std::make_unique<SparkMaxSlcanDevice>();
            if (dev->open(port, slcan_cfg_.bitrate_code)) {
                RCLCPP_INFO(logger_, "[discovery] connected SLCAN on %s (configured)", port.c_str());
                std::lock_guard<std::mutex> lock(mutex_);
                slcan_ = std::move(dev);
                slcan_missing = false;
                claimed_ports.insert(port);
            } else {
                RCLCPP_WARN(logger_, "[discovery] configured SLCAN port %s did not open",
                    port.c_str());
            }
        }
    }

    // ── Phase 2: probe each unclaimed port for an Arduino ─────────────────
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
            RCLCPP_DEBUG(logger_, "[discovery] %s on %s — not wanted, releasing",
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

    // ── Phase 3: probe remaining unclaimed ports for SLCAN (auto mode) ────
    if (slcan_missing && slcan_cfg_.port == "auto") {
        for (const auto & port : candidate_ports) {
            if (stop_flag_) return;
            if (claimed_ports.count(port)) continue;

            auto dev = std::make_unique<SparkMaxSlcanDevice>();
            if (dev->open(port, slcan_cfg_.bitrate_code)) {
                RCLCPP_INFO(logger_, "[discovery] connected SLCAN on %s (auto)", port.c_str());
                std::lock_guard<std::mutex> lock(mutex_);
                slcan_ = std::move(dev);
                claimed_ports.insert(port);
                break;
            }
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

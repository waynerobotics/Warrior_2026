#include <glob.h>

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include "warrior_driver/swerve/device_registry.hpp"
#include "warrior_driver/sparkmax/serial_protocol.hpp"

namespace warrior::driver::colors {
    constexpr const char* RED     = "\033[1;31m";
    constexpr const char* GREEN   = "\033[1;32m";
    constexpr const char* YELLOW  = "\033[1;33m";
    constexpr const char* BLUE    = "\033[1;34m";
    constexpr const char* MAGENTA = "\033[1;35m";
    constexpr const char* CYAN    = "\033[1;36m";
    constexpr const char* RESET   = "\033[0m";
}

namespace warrior::driver {

using namespace colors;

namespace {

constexpr auto ARDUINO_RESET_DELAY     = std::chrono::milliseconds(2000);
constexpr auto HANDSHAKE_TIMEOUT       = std::chrono::milliseconds(3000);
constexpr auto SPARK_DISCOVER_TIMEOUT  = std::chrono::milliseconds(3000);

/**
 * Glob for all /dev/ttyACM* and /dev/ttyUSB* ports and return them sorted.
 * ttyS* (hardware UARTs) are intentionally excluded — they are slow to open
 * and are never our devices.
 *
 * @return Sorted list of candidate port paths.
 */
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

// ── Construction / destruction ────────────────────────────────────────────

/**
 * Construct a DeviceRegistry with the given hardware configs.
 * Does not open any ports or start the discovery thread — call start() for that.
 *
 * @param logger            ROS2 logger to use for all log output.
 * @param arduino_cfg       Names of Arduino devices to discover.
 * @param spark_cfg         CAN IDs of SPARK MAX devices to discover.
 * @param discovery_period  How often the discovery loop rescans for missing devices.
 */
DeviceRegistry::DeviceRegistry(rclcpp::Logger logger,
                               ArduinoConfig arduino_cfg,
                               SparkConfig spark_cfg,
                               std::chrono::duration<double> discovery_period)
: logger_(std::move(logger))
, arduino_cfg_(std::move(arduino_cfg))
, spark_cfg_(std::move(spark_cfg))
, discovery_period_(discovery_period)
{
    std::cout << GREEN << "[DeviceRegistry]" << RESET << " "
              << "configured with discovery_period = "
              << discovery_period_.count() << "s, "
              << arduino_cfg_.wanted_names.size() << " wanted Arduino(s), "
              << spark_cfg_.wanted_can_ids.size() << " wanted SPARK MAX(s)"
              << std::endl;
}

/**
 * Destructor. Calls stop() to join the discovery thread and close all devices.
 */
DeviceRegistry::~DeviceRegistry()
{
    stop();
}

// ── Lifecycle ─────────────────────────────────────────────────────────────

/**
 * Start the background discovery thread.
 * No-op if the thread is already running.
 */
void DeviceRegistry::start()
{
    if (discovery_thread_.joinable()) return;
    stop_flag_ = false;
    discovery_thread_ = std::thread(&DeviceRegistry::discovery_loop, this);
}

/**
 * Stop the background discovery thread and close all open devices.
 * Blocks until the discovery thread has joined. Safe to call multiple times.
 */
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

/**
 * @return True if an Arduino with the given @p name is currently connected.
 */
bool DeviceRegistry::is_arduino_connected(const std::string & name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return arduinos_.find(name) != arduinos_.end();
}

/**
 * @return Sorted list of names of all currently connected Arduinos.
 */
std::vector<std::string> DeviceRegistry::connected_arduino_names()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> names;
    names.reserve(arduinos_.size());
    for (const auto & [name, _] : arduinos_) names.push_back(name);
    std::sort(names.begin(), names.end());
    return names;
}

/**
 * Send a drive percent command to the Arduino with the given @p name.
 * On write failure the device is dropped and must be rediscovered.
 *
 * @param name     Arduino device name (e.g. "front_left").
 * @param percent  Drive power in the range [-100, 100].
 * @return         True on success, false if the device is not connected or write failed.
 */
bool DeviceRegistry::send_drive_percent(const std::string & name, int percent)
{
    const std::string frame = serial_protocol::encode_drive(name, percent);

    std::lock_guard<std::mutex> lock(mutex_);
    auto it = arduinos_.find(name);
    if (it == arduinos_.end()) return false;
    if (!it->second->write_frame(frame)) {
        RCLCPP_WARN(
            logger_,
            "%s[%s]%s %swrite failed%s; dropping connection on %s%s%s",
            RED, name.c_str(), RESET,
            RED, RESET,
            CYAN, it->second->port().c_str(), RESET);

        arduinos_.erase(it);
        return false;
    }
    return true;
}

/**
 * Drain all pending lines from every connected Arduino and return them.
 * Devices that have closed themselves (e.g. USB disconnect) are removed.
 *
 * @return Vector of (device_name, line) pairs in arrival order.
 */
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

/**
 * Send a zero drive command to every currently connected Arduino.
 * Used as a safe-stop on shutdown.
 */
void DeviceRegistry::send_zero_to_all_arduinos()
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & [name, dev] : arduinos_) {
        dev->write_frame(serial_protocol::encode_drive(name, 0));
    }
}

// ── SPARK MAX API ────────────────────────────────────────────────────────

/**
 * @return True if a SPARK MAX with the given @p can_id is connected and its session is open.
 */
bool DeviceRegistry::is_spark_connected(int can_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    return it != sparks_.end() && it->second && it->second->is_open();
}

/**
 * @return Sorted list of CAN IDs of all currently connected SPARK MAXes.
 */
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

/**
 * @return The serial port path for the SPARK MAX with the given @p can_id,
 *         or an empty string if not connected.
 */
std::string DeviceRegistry::spark_port(int can_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    if (it == sparks_.end() || !it->second) return {};
    return it->second->port();
}

/**
 * Command the SPARK MAX at @p can_id to hold a position setpoint.
 * Enables the controller if it is not already enabled.
 *
 * @param can_id            CAN device ID of the target SPARK MAX.
 * @param motor_rotations   Target position in motor rotations.
 * @return                  True on success, false if the device is not connected.
 */
bool DeviceRegistry::send_steer_position(int can_id, float motor_rotations)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    if (it == sparks_.end() || !it->second || !it->second->is_open()) return false;
    it->second->set_target_position(motor_rotations);
    it->second->enable();
    return true;
}

/**
 * Stop sending position setpoints to the SPARK MAX at @p can_id.
 * The session's heartbeat continues so status frames keep flowing.
 *
 * @param can_id  CAN device ID of the target SPARK MAX.
 */
void DeviceRegistry::release_steer(int can_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    if (it == sparks_.end() || !it->second) return;
    it->second->disable();
}

/**
 * @return The latest position reading from the SPARK MAX at @p can_id in motor
 *         rotations, or nullopt if not connected or the value is NaN.
 */
std::optional<float> DeviceRegistry::spark_position_rot(int can_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    if (it == sparks_.end() || !it->second) return std::nullopt;
    const float p = it->second->position_rotations();
    if (std::isnan(p)) return std::nullopt;
    return p;
}

/**
 * @return The latest applied output percentage from the SPARK MAX at @p can_id,
 *         or nullopt if not connected.
 */
std::optional<float> DeviceRegistry::spark_applied_pct(int can_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sparks_.find(can_id);
    if (it == sparks_.end() || !it->second) return std::nullopt;
    return it->second->applied_output_percent();
}

/**
 * Return how many seconds have elapsed since the last Status 2 (position)
 * frame was received from the SPARK MAX at @p can_id.
 *
 * Internally tracks the Status 2 frame counter; when it increments the
 * timestamp is updated. This avoids relying on a wall-clock timestamp inside
 * the session which would require an extra synchronisation point.
 *
 * @param can_id  CAN device ID of the target SPARK MAX.
 * @param now     Current ROS time (from the calling node).
 * @return        Age in seconds, or -1.0 if the device is not connected or
 *                no Status 2 frame has ever been seen.
 */
double DeviceRegistry::seconds_since_spark_position(int can_id, rclcpp::Time now)
{
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

// ── Discovery helpers ─────────────────────────────────────────────────────

/**
 * @return Names of wanted Arduinos that are not currently in arduinos_.
 *         Caller must hold mutex_.
 */
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

/**
 * @return CAN IDs of wanted SPARK MAXes that are not currently in sparks_.
 *         Caller must hold mutex_.
 */
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

/**
 * @return Port paths of all currently open Arduino and SPARK MAX connections.
 *         Used by scan_once() to avoid probing already-claimed ports.
 *         Caller must hold mutex_.
 */
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

/**
 * Block for ARDUINO_RESET_DELAY, waking early if stop_flag_ is set.
 * Called after opening an Arduino port to wait out the DTR-triggered reset.
 *
 * @return True if stop was requested during the wait (caller should abort).
 */
bool DeviceRegistry::wait_arduino_reset()
{
    std::unique_lock<std::mutex> lock(stop_mutex_);
    return stop_cv_.wait_for(lock, ARDUINO_RESET_DELAY,
                             [this] { return stop_flag_.load(); });
}

/**
 * Open @p port, wait for the Arduino reset delay, then run the WHO/NAME
 * handshake. On success returns the device name; on failure or timeout
 * returns nullopt and leaves the port closed.
 *
 * @param port  Serial port path, e.g. "/dev/ttyACM0".
 * @param dev   Device object to open (caller retains ownership).
 * @return      Discovered Arduino name, or nullopt on failure.
 */
std::optional<std::string> DeviceRegistry::probe_arduino(const std::string & port,
                                                          ArduinoSerialDevice & dev)
{
    if (!dev.open(port, arduino_cfg_.baud_rate)) return std::nullopt;
    if (wait_arduino_reset()) {
        dev.close();
        return std::nullopt;
    }
    return dev.handshake(HANDSHAKE_TIMEOUT);
}

/**
 * Open a SPARK MAX session on @p port and wait for it to identify itself
 * via passive CAN status frames. Returns the session if a valid device_id
 * is seen within SPARK_DISCOVER_TIMEOUT, otherwise closes and returns nullptr.
 *
 * The returned session is fully running with its own tx/rx threads; the
 * caller just needs to keep it alive and call close() when done.
 *
 * @param port  Serial port path of the SPARK MAX SLCAN adapter.
 * @return      Running SparkMaxSession, or nullptr on failure / timeout.
 */
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
        sess->close();
        return nullptr;
    }
    return sess;
}

/**
 * Perform one discovery sweep across all USB ports.
 *
 * Checks which Arduino names and SPARK MAX CAN IDs are still missing,
 * then scans available ports in two phases:
 *   1. SPARK MAXes  — matched by CAN device_id from passive status frames.
 *   2. Arduinos     — matched by WHO/NAME handshake.
 *
 * Already-claimed ports are skipped. If a SPARK MAX stays silent during
 * probing, its CAN ID is assumed from the front of the missing list as a
 * fallback. Devices not in the wanted lists are released immediately.
 *
 * Returns early if stop_flag_ is set at any point during scanning.
 */
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

    RCLCPP_INFO(
        logger_,
        "%s[discovery]%s scanning: %s%zu Arduino(s) missing%s, %s%zu SPARK(s) missing%s",
        YELLOW, RESET,
        RED, missing_arduinos.size(), RESET,
        MAGENTA, missing_sparks.size(), RESET);

    // ── Phase 1: SPARK MAXes ─────────────────────────────────────────────
    if (!missing_sparks.empty()) {
        const auto spark_ports = list_sparkmax_ports();
        for (const auto & port : spark_ports) {
            if (stop_flag_) return;
            if (claimed_ports.count(port)) continue;
            if (missing_sparks.empty()) break;

            auto sess = probe_spark(port);
            if (!sess) continue;
            int dev = sess->device_id();

            // If the controller stayed silent during probing, fall back to
            // the first missing CAN ID and force it onto the session.
            if (dev < 0) {
                dev = missing_sparks.front();
                sess->force_device_id(dev);
                RCLCPP_WARN(
                    logger_,
                    "%s[discovery]%s %sSPARK%s on %s%s%s stayed silent; using fallback CAN ID %d",
                    YELLOW, RESET,
                    MAGENTA, RESET,
                    CYAN, port.c_str(), RESET,
                    dev);
            }

            auto want_it = std::find(missing_sparks.begin(), missing_sparks.end(), dev);
            if (want_it == missing_sparks.end()) {
                // Device is present but not in our wanted list — close and move on.
                RCLCPP_DEBUG(
                    logger_,
                    "%s[discovery]%s %sSPARK dev=%d%s on %s%s%s — %snot wanted, releasing%s",
                    YELLOW, RESET,
                    MAGENTA, dev, RESET,
                    CYAN, port.c_str(), RESET,
                    RED, RESET);

                sess->close();
                continue;
            }

            RCLCPP_INFO(
                logger_,
                "%s[discovery]%s %sconnected%s %sSPARK dev=%d%s on %s%s%s",
                YELLOW, RESET,
                GREEN, RESET,
                MAGENTA, dev, RESET,
                CYAN, port.c_str(), RESET);

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
                // Arduino responded but its name is not in our wanted list — skip it.
                RCLCPP_DEBUG(
                    logger_,
                    "%s[discovery]%s %sArduino %s%s on %s%s%s — %snot wanted, releasing%s",
                    YELLOW, RESET,
                    BLUE, name.c_str(), RESET,
                    CYAN, port.c_str(), RESET,
                    RED, RESET);

                continue;
            }

            RCLCPP_INFO(
                logger_,
                "%s[discovery]%s %sconnected%s %sArduino %s%s on %s%s%s",
                YELLOW, RESET,
                GREEN, RESET,
                BLUE, name.c_str(), RESET,
                CYAN, port.c_str(), RESET);

            {
                std::lock_guard<std::mutex> lock(mutex_);
                arduinos_[name] = std::move(dev);
            }
            missing_arduinos.erase(wanted_it);
            claimed_ports.insert(port);
        }
    }
}

/**
 * Background thread entry point. Calls scan_once() in a loop, sleeping for
 * discovery_period_ between sweeps. Exits when stop_flag_ is set.
 */
void DeviceRegistry::discovery_loop()
{
    while (!stop_flag_) {
        scan_once();
        std::unique_lock<std::mutex> lock(stop_mutex_);
        stop_cv_.wait_for(lock, discovery_period_, [this] { return stop_flag_.load(); });
    }
}

}  // namespace warrior::driver
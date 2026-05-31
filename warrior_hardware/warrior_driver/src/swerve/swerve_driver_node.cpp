#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "warrior_driver/arduino/arduino_serial_device.hpp"
#include "warrior_driver/sparkmax/sparkmax_frame.hpp"
#include "warrior_driver/swerve/unit_conversions.hpp"
#include "warrior_driver/swerve/swerve_driver_node.hpp"
#include "warrior_driver/arduino/serial_protocol.hpp"

namespace warrior::driver {

namespace {

// ── Small helpers to build diagnostic KeyValue pairs of various types ──
diagnostic_msgs::msg::KeyValue kv(const std::string & key, std::string value)
{
    diagnostic_msgs::msg::KeyValue out;
    out.key   = key;
    out.value = std::move(value);
    return out;
}
diagnostic_msgs::msg::KeyValue kv(const std::string & key, bool value)
{
    return kv(key, std::string(value ? "true" : "false"));
}
diagnostic_msgs::msg::KeyValue kv(const std::string & key, double value)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.4f", value);
    return kv(key, std::string(buf));
}
diagnostic_msgs::msg::KeyValue kv(const std::string & key, int value)
{
    return kv(key, std::to_string(value));
}

}  // namespace

// ════════════════════════════════════════════════════════════════════════
//  Construction
// ════════════════════════════════════════════════════════════════════════
SwerveDriverNode::SwerveDriverNode() : rclcpp::Node("warrior_swerve_driver")
{
    // ── Core topic / timing parameters ──
    command_topic_       = declare_parameter<std::string>("command_topic", "/warrior_swerve_command");
    state_topic_         = declare_parameter<std::string>("state_topic",   "/warrior_swerve_state");
    update_rate_hz_      = declare_parameter<double>("update_rate_hz", 50.0);
    command_timeout_s_   = declare_parameter<double>("command_timeout_s", 0.5);
    steer_stale_after_s_ = declare_parameter<double>("steer_stale_after_s", 0.5);
    discovery_period_s_  = declare_parameter<double>("discovery_period_s", 2.0);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 1.0);
    baud_rate_           = declare_parameter<int>("arduino.baud_rate", 115200);

    // ── Auto-calibration parameters ──
    calib_timeout_s_  = declare_parameter<double>("calib.timeout_s", 60.0);
    calib_samples_    = declare_parameter<int>("calib.samples", 100);
    calib_write_path_ = declare_parameter<std::string>("calib.write_path", "");

    // Declared but unused — kept so existing YAML configs don't generate
    // "parameter not declared" warnings while folks migrate.
    (void) declare_parameter<std::string>("sparkmax.slcan_interface", "auto");
    (void) declare_parameter<std::string>("sparkmax.bitrate_code", "8");

    load_modules();   // populate modules_ and module_index_ from parameters

    // Mark the start of the calibration window (used for the connect timeout).
    calib_start_time_ = this->now();

    // ── Build the Arduino discovery config from module drive device names ──
    DeviceRegistry::ArduinoConfig arduino_cfg;
    arduino_cfg.baud_rate = baud_rate_;
    arduino_cfg.wanted_names.reserve(modules_.size());
    for (const auto & m : modules_) {
        if (!m.config.drive_device_name.empty()) {
            arduino_cfg.wanted_names.push_back(m.config.drive_device_name);
        }
    }

    // ── Build the SPARK MAX discovery config from module CAN IDs ──
    DeviceRegistry::SparkConfig spark_cfg;
    spark_cfg.wanted_can_ids.reserve(modules_.size());
    for (const auto & m : modules_) {
        if (m.config.spark_can_id > 0) {
            spark_cfg.wanted_can_ids.push_back(m.config.spark_can_id);
        }
    }

    // ── Start the device registry (spawns the background discovery thread) ──
    registry_ = std::make_unique<DeviceRegistry>(
        get_logger(), std::move(arduino_cfg), std::move(spark_cfg),
        std::chrono::duration<double>(discovery_period_s_));
    registry_->start();

    // ── Subscribe to incoming commands ──
    cmd_sub_ = create_subscription<warrior_msgs::msg::SwerveCmd>(
        command_topic_, rclcpp::QoS(10),
        std::bind(&SwerveDriverNode::on_command, this, std::placeholders::_1));

    // ── Publisher for per-module state ──
    state_pub_ = create_publisher<warrior_msgs::msg::SwerveState>(
        state_topic_, rclcpp::QoS(10));

    // ── Publisher for diagnostics ──
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10));

    // ── Main update timer ──
    const auto update_period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    update_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(update_period),
        std::bind(&SwerveDriverNode::update, this));

    // ── Diagnostics timer ──
    const auto diag_period = std::chrono::duration<double>(1.0 / diagnostics_rate_hz_);
    diag_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(diag_period),
        std::bind(&SwerveDriverNode::publish_diagnostics, this));

    RCLCPP_INFO(get_logger(),
        "warrior_swerve_driver up: %zu modules, sub=%s, pub=%s, rate=%.1f Hz, "
        "timeout=%.2fs, steer_stale=%.2fs, baud=%d, scan=%.1fs, diag=%.1f Hz | "
        "calib: samples=%d, timeout=%.0fs, write_path='%s'",
        modules_.size(), command_topic_.c_str(), state_topic_.c_str(),
        update_rate_hz_, command_timeout_s_, steer_stale_after_s_, baud_rate_,
        discovery_period_s_, diagnostics_rate_hz_,
        calib_samples_, calib_timeout_s_, calib_write_path_.c_str());
}

SwerveDriverNode::~SwerveDriverNode()
{
    if (registry_) {
        registry_->stop();
    }
}

// ════════════════════════════════════════════════════════════════════════
//  Auto-calibration
//
//  Runs once per update() tick until calib_done_ becomes true. For each
//  module that has not yet recorded its forward offset:
//    1. Wait for the SPARK MAX to be connected.
//    2. Wait for fresh encoder data.
//    3. Accumulate raw encoder readings into calib_buffer.
//    4. Once calib_samples_ readings are gathered, average them, store the
//       result in config.encoder_pos_forward, and mark the module recorded.
//  When every module is recorded, write the YAML and set calib_done_.
//  If calib_timeout_s_ elapses before all modules connect, this logs FATAL
//  and shuts the node down.
// ════════════════════════════════════════════════════════════════════════
void SwerveDriverNode::auto_calibrate()
{
    const auto now = this->now();

    // ── Hard deadline ──
    const double elapsed = (now - calib_start_time_).seconds();
    if (elapsed > calib_timeout_s_) {
        RCLCPP_FATAL(get_logger(),
            "[calib] Timeout after %.0fs — not all modules calibrated. Shutting down.",
            calib_timeout_s_);
        for (const auto & module : modules_) {
            if (!module.calib_recorded) {
                RCLCPP_FATAL(get_logger(), "[calib]   module '%s' never completed calibration",
                    module.config.name.c_str());
            }
        }
        rclcpp::shutdown();
        return;
    }

    bool all_recorded = true;

    for (auto & module : modules_) {
        if (module.calib_recorded) continue;
        all_recorded = false;

        const auto & cfg = module.config;

        // ── 1. SPARK MAX connected? ──
        const bool steer_ok = registry_ && registry_->is_spark_connected(cfg.spark_can_id);
        if (!steer_ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[calib] '%s' waiting for SPARK MAX (CAN %d)... (%.0f/%.0fs)",
                cfg.name.c_str(), cfg.spark_can_id, elapsed, calib_timeout_s_);
            continue;
        }

        // ── 2. Fresh encoder data? ──
        const auto pos_rot = registry_->spark_position_rot(cfg.spark_can_id);
        const double pos_age = registry_->seconds_since_spark_position(cfg.spark_can_id, now);

        // ── DEBUG: log everything we see to a txt file, even rejected samples ──
        {
            std::ofstream dbg("/tmp/calib_debug.txt", std::ios::app);
            if (dbg.is_open()) {
                dbg << "[" << std::fixed << std::setprecision(3) << now.seconds() << "] "
                    << cfg.name
                    << " can=" << cfg.spark_can_id
                    << " has_value=" << (pos_rot.has_value() ? "1" : "0")
                    << " pos_rot=" << (pos_rot.has_value() ? std::to_string(*pos_rot) : "nan")
                    << " pos_age=" << std::setprecision(4) << pos_age
                    << " stale_thresh=" << steer_stale_after_s_
                    << " buffer_size=" << module.calib_buffer.size()
                    << "\n";
            }
        }

        if (!pos_rot.has_value() || pos_age < 0.0 || pos_age >= steer_stale_after_s_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[calib] '%s' connected; waiting for fresh encoder data "
                "(has_value=%d, age=%.4f, thresh=%.2f)...",
                cfg.name.c_str(), pos_rot.has_value() ? 1 : 0, pos_age, steer_stale_after_s_);
            continue;
        }

        // ── 3. Accumulate sample ──
        module.calib_buffer.push_back(static_cast<double>(*pos_rot));

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "[calib] '%s' collecting: %zu/%d samples (latest=%.4f rot)",
            cfg.name.c_str(), module.calib_buffer.size(), calib_samples_,
            static_cast<double>(*pos_rot));

        // ── 4. Window full → average and finalize ──
        if (static_cast<int>(module.calib_buffer.size()) >= calib_samples_) {
            double sum = 0.0;
            for (double v : module.calib_buffer) sum += v;
            const double avg = sum / static_cast<double>(module.calib_buffer.size());

            double var = 0.0;
            for (double v : module.calib_buffer) var += (v - avg) * (v - avg);
            var /= static_cast<double>(module.calib_buffer.size());
            const double stddev = std::sqrt(var);

            // ── DEBUG: dump the full buffer for this module ──
            {
                std::ofstream dbg("/tmp/calib_debug.txt", std::ios::app);
                if (dbg.is_open()) {
                    dbg << "===== '" << cfg.name << "' FINAL: avg=" << std::setprecision(4) << avg
                        << " stddev=" << stddev << " n=" << module.calib_buffer.size() << " =====\n";
                    dbg << "  samples: ";
                    for (double v : module.calib_buffer) dbg << v << " ";
                    dbg << "\n";
                }
            }

            module.config.encoder_pos_forward = avg;
            module.calib_recorded = true;

            RCLCPP_INFO(get_logger(),
                "[calib] '%s' DONE: encoder_pos_forward = %.4f rot (avg over %d, stddev=%.4f)",
                cfg.name.c_str(), avg, calib_samples_, stddev);

            if (stddev > 0.1) {
                RCLCPP_WARN(get_logger(),
                    "[calib] '%s' high stddev (%.4f) — wheel may have moved during calibration!",
                    cfg.name.c_str(), stddev);
            }
        }
    }

    if (all_recorded) {
        calib_done_ = true;
        RCLCPP_INFO(get_logger(), "[calib] ===== All modules calibrated =====");
        for (const auto & module : modules_) {
            RCLCPP_INFO(get_logger(), "[calib]   %-6s encoder_pos_forward = %.4f rot",
                module.config.name.c_str(), module.config.encoder_pos_forward);
        }
        write_calibration_yaml();
        RCLCPP_INFO(get_logger(), "[calib] ===== Entering normal operation =====");
    }
}

// ════════════════════════════════════════════════════════════════════════
//  Write calibration results to YAML
//
//  Produces a small YAML file that, when loaded after warrior_driver.yaml,
//  overrides each module's encoder_pos_forward. A timestamp comment records
//  when the calibration was performed. This file is auto-generated and not
//  meant to be hand-edited.
// ════════════════════════════════════════════════════════════════════════
void SwerveDriverNode::write_calibration_yaml()
{
    if (calib_write_path_.empty()) {
        RCLCPP_WARN(get_logger(),
            "[calib] calib.write_path is empty — skipping YAML write "
            "(calibration values are kept in memory for this run only)");
        return;
    }

    // Human-readable local timestamp for the comment header.
    const std::time_t t = std::time(nullptr);
    char timestamp[64];
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

    std::ofstream f(calib_write_path_);
    if (!f.is_open()) {
        RCLCPP_ERROR(get_logger(),
            "[calib] Failed to open '%s' for writing — calibration not persisted!",
            calib_write_path_.c_str());
        return;
    }

    f << "# ============================================================\n";
    f << "# Auto-generated steer calibration\n";
    f << "# Calibrated: " << timestamp << "\n";
    f << "# DO NOT EDIT BY HAND — regenerated on every launch with auto-calibration.\n";
    f << "# Load this AFTER warrior_driver.yaml to override encoder_pos_forward.\n";
    f << "# ============================================================\n";
    f << "warrior_driver:\n";
    f << "  ros__parameters:\n";
    f << "    modules:\n";
    for (const auto & module : modules_) {
        f << "      " << module.config.name << ":\n";
        f << "        encoder_pos_forward: "
          << std::fixed << std::setprecision(4) << module.config.encoder_pos_forward
          << "  # calibrated " << timestamp << "\n";
    }
    f.close();

    RCLCPP_INFO(get_logger(),
        "[calib] Wrote calibration to '%s'", calib_write_path_.c_str());
}

// ════════════════════════════════════════════════════════════════════════
//  Safe stop (called on shutdown)
// ════════════════════════════════════════════════════════════════════════
void SwerveDriverNode::send_safe_stop()
{
    if (registry_) {
        RCLCPP_INFO(get_logger(), "shutdown: sending drive 0%% to all connected Arduinos");
        registry_->send_zero_to_all_arduinos();
    }
}

// ════════════════════════════════════════════════════════════════════════
//  Load module configs from YAML parameters
// ════════════════════════════════════════════════════════════════════════
void SwerveDriverNode::load_modules()
{
    const auto names = declare_parameter<std::vector<std::string>>(
        "module_names", std::vector<std::string>{});

    if (names.empty()) {
        RCLCPP_ERROR(get_logger(),
            "Parameter 'module_names' is empty. Configure modules via YAML.");
        return;
    }

    modules_.reserve(names.size());
    for (const auto & name : names) {
        SwerveModuleConfig cfg;
        cfg.name                            = name;
        cfg.drive_device_name               = declare_parameter<std::string>("modules." + name + ".drive_device_name", "");
        cfg.steer_device_name               = declare_parameter<std::string>("modules." + name + ".steer_device_name", "");
        cfg.spark_can_id                    = declare_parameter<int>(        "modules." + name + ".spark_can_id", 0);
        cfg.steer_motor_rot_per_module_rot  = declare_parameter<double>(     "modules." + name + ".steer_motor_rot_per_module_rot", 1.0);
        cfg.steer_offset_rad                = declare_parameter<double>(     "modules." + name + ".steer_offset_rad", 0.0);
        cfg.encoder_pos_forward             = declare_parameter<double>(     "modules." + name + ".encoder_pos_forward", 0.0);
        cfg.steer_sign                      = declare_parameter<double>(     "modules." + name + ".steer_sign", 1.0);
        cfg.drive_sign                      = declare_parameter<double>(     "modules." + name + ".drive_sign", 1.0);
        cfg.max_drive_rad_s                 = declare_parameter<double>(     "modules." + name + ".max_drive_rad_s", 1.0);

        module_index_[name] = modules_.size();
        SwerveModule m;
        m.config = cfg;
        m.state.last_update_time  = this->now();
        m.state.last_command_time = this->now();
        modules_.push_back(std::move(m));

        RCLCPP_INFO(get_logger(),
            "module '%s': drive=%s, steer=%s (CAN %d), gear=%.3f, offset=%.3f rad, "
            "encoder_pos_forward=%.4f rot, signs=(s%+.0f,d%+.0f), max_drive=%.2f rad/s",
            cfg.name.c_str(), cfg.drive_device_name.c_str(), cfg.steer_device_name.c_str(),
            cfg.spark_can_id, cfg.steer_motor_rot_per_module_rot, cfg.steer_offset_rad,
            cfg.encoder_pos_forward, cfg.steer_sign, cfg.drive_sign, cfg.max_drive_rad_s);
    }
}

// ════════════════════════════════════════════════════════════════════════
//  Module lookup by name
// ════════════════════════════════════════════════════════════════════════
SwerveModule * SwerveDriverNode::find_module(const std::string & name)
{
    auto it = module_index_.find(name);
    if (it == module_index_.end()) return nullptr;
    return &modules_[it->second];
}

// ════════════════════════════════════════════════════════════════════════
//  Command callback: store the latest command for the addressed module
// ════════════════════════════════════════════════════════════════════════
void SwerveDriverNode::on_command(const warrior_msgs::msg::SwerveCmd::SharedPtr msg)
{
    SwerveModule *module = find_module(msg->swerve_id);
    if (!module) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Ignoring command for unknown swerve_id '%s'", msg->swerve_id.c_str());
        return;
    }
    module->state.cmd_steer_position_rad   = msg->steer_position_rad;
    module->state.cmd_drive_velocity_rad_s = msg->drive_velocity_rad_s;
    module->state.have_command             = true;
    module->state.last_command_time        = this->now();
}

// ════════════════════════════════════════════════════════════════════════
//  Drain and log pending Arduino lines (ACK / ERR / other)
// ════════════════════════════════════════════════════════════════════════
void SwerveDriverNode::drain_and_log_arduino_messages()
{
    if (!registry_) return;
    for (const auto & [device_name, line] : registry_->drain_arduino_messages()) {
        const auto fields = warrior::driver::serial_protocol::parse_frame(line);
        if (!fields || fields->empty()) continue;
        const std::string & type = fields->at(0);
        if (type == "ACK") {
            RCLCPP_DEBUG(get_logger(), "[%s] ACK %s",
                device_name.c_str(),
                fields->size() > 1 ? fields->at(1).c_str() : "");
        } else if (type == "ERR") {
            RCLCPP_WARN(get_logger(), "[%s] ERR %s",
                device_name.c_str(),
                fields->size() > 1 ? fields->at(1).c_str() : "(no reason)");
        } else {
            RCLCPP_DEBUG(get_logger(), "[%s] %s", device_name.c_str(), line.c_str());
        }
    }
}

// ════════════════════════════════════════════════════════════════════════
//  Main periodic update
//
//  Calibration gate runs first: until every module's forward offset has been
//  recorded, this loop only calibrates and returns — no commands are sent.
//  After calibration, each module:
//    - maps its steer command to an absolute encoder target and sends it,
//    - maps its drive command to a percentage and sends it,
//    - reads the latest encoder feedback (mapped back to algorithm coords),
//    - integrates an open-loop drive position,
//    - publishes its state.
// ════════════════════════════════════════════════════════════════════════
void SwerveDriverNode::update()
{
    const auto now = this->now();
    drain_and_log_arduino_messages();

    // ── Calibration gate: don't accept/send commands until calibrated ──
    if (!calib_done_) {
        auto_calibrate();
        return;
    }

    for (auto & module : modules_) {
        const auto & cfg = module.config;
        auto & state = module.state;

        // A command is "timed out" if it's older than command_timeout_s_.
        const bool timed_out = state.have_command &&
            (now - state.last_command_time).seconds() > command_timeout_s_;

        // Steer keeps its last commanded position even when timed out (the
        // wheel should hold its heading); drive is zeroed on timeout for safety.
        const double steer_cmd = state.cmd_steer_position_rad;
        const double drive_cmd = (state.have_command && !timed_out) ? state.cmd_drive_velocity_rad_s : 0.0;

        // ── Map algorithm-coordinate commands to hardware units ──
        const double encoder_position = steer_rad_to_encoder_pos(steer_cmd, cfg);
        const int    drive_percent    = drive_rad_s_to_percent(drive_cmd, cfg);

        // ── Drive path (Arduino) ──
        const bool drive_connected =
            registry_ && registry_->is_arduino_connected(cfg.drive_device_name);

        std::string drive_status;
        if (!drive_connected) {
            drive_status = "scanning";
        } else if (timed_out) {
            drive_status = "timeout";
            registry_->send_drive_percent(cfg.drive_device_name, 0);
        } else if (state.have_command) {
            const bool ok = registry_->send_drive_percent(cfg.drive_device_name, drive_percent);
            drive_status = ok ? "active" : "write_failed";
        } else {
            drive_status = "idle";
            registry_->send_drive_percent(cfg.drive_device_name, 0);
        }

        // ── Steer path (SPARK MAX over per-USB SLCAN) ──
        const bool spark_connected =
            registry_ && registry_->is_spark_connected(cfg.spark_can_id);

        // Pull the latest absolute encoder reading and map it back to algorithm
        // coordinates, but only if the data is fresh.
        if (spark_connected) {
            const auto pos_rot = registry_->spark_position_rot(cfg.spark_can_id);
            const double pos_age =
                registry_->seconds_since_spark_position(cfg.spark_can_id, now);
            if (pos_rot.has_value() && pos_age >= 0.0
                && pos_age < steer_stale_after_s_)
            {
                state.fb_steer_position_rad =
                    encoder_pos_to_steer_rad(static_cast<double>(*pos_rot), cfg);
                state.last_steer_pos_time = now - rclcpp::Duration::from_seconds(pos_age);
            }
        }

        const bool steer_pos_fresh =
            state.last_steer_pos_time.nanoseconds() > 0 &&
            (now - state.last_steer_pos_time).seconds() < steer_stale_after_s_;

        std::string steer_status;
        bool steer_connected_now = false;
        if (!spark_connected) {
            steer_status = "scanning";
        } else if (state.have_command && !timed_out) {
            const bool ok = registry_->send_steer_position(
                cfg.spark_can_id, static_cast<float>(encoder_position));
            steer_connected_now = ok && steer_pos_fresh;
            if (!ok) {
                steer_status = "write_failed";
            } else if (!steer_pos_fresh) {
                steer_status = "no_feedback";
            } else {
                steer_status = "active";
            }
        } else {
            // No command (or it timed out): stop driving setpoints but keep the
            // session heartbeat going so status frames keep flowing.
            registry_->release_steer(cfg.spark_can_id);
            steer_status = steer_pos_fresh ? "idle" : "no_feedback";
            steer_connected_now = steer_pos_fresh;
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[%s] steer cmd=%.3f rad / fb=%.3f rad (%s) | drive cmd=%.3f rad/s -> %d%% (%s)",
            cfg.name.c_str(),
            steer_cmd, state.fb_steer_position_rad, steer_status.c_str(),
            drive_cmd, drive_percent, drive_status.c_str());

        // ── Drive position integration (open-loop, no encoder) ──
        const double dt = (now - state.last_update_time).seconds();
        state.last_update_time = now;

        // Guard against large jumps (e.g. from pausing in a debugger).
        if (dt > 0.0 && dt < 1.0) {
            state.fb_drive_position_rad += drive_cmd * dt;
        }

        // ── Publish module state ──
        warrior_msgs::msg::SwerveState state_msg;
        state_msg.swerve_id            = cfg.name;
        state_msg.steer_position_rad   = state.fb_steer_position_rad;
        state_msg.steer_velocity_rad_s = state.fb_steer_velocity_rad_s;
        // Drive velocity has no encoder feedback here; echo the command so
        // consumers can treat it as open-loop.
        state_msg.drive_position_rad   = state.fb_drive_position_rad;
        state_msg.drive_velocity_rad_s = drive_cmd;
        state_msg.steer_connected      = steer_connected_now;
        state_msg.drive_connected      = drive_connected;
        state_msg.steer_status         = steer_status;
        state_msg.drive_status         = drive_status;
        state_msg.stamp                = now;
        state_pub_->publish(state_msg);
    }
}

// ════════════════════════════════════════════════════════════════════════
//  Diagnostics
// ════════════════════════════════════════════════════════════════════════
void SwerveDriverNode::publish_diagnostics()
{
    if (!registry_) return;
    const auto now = this->now();

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now;

    // ── Per-module status ──
    for (const auto & module : modules_) {
        const auto & cfg = module.config;
        const auto & state = module.state;

        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name        = "warrior_swerve_driver: " + cfg.name;
        st.hardware_id = cfg.drive_device_name + " / spark can=" + std::to_string(cfg.spark_can_id);

        const bool drive_ok = registry_->is_arduino_connected(cfg.drive_device_name);
        const double steer_pos_age = state.last_steer_pos_time.nanoseconds() > 0
            ? (now - state.last_steer_pos_time).seconds() : -1.0;
        const bool steer_fresh = steer_pos_age >= 0.0 && steer_pos_age < steer_stale_after_s_;
        const bool cmd_recent  = state.have_command &&
                                 (now - state.last_command_time).seconds() < command_timeout_s_;

        if (drive_ok && steer_fresh) {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
            st.message = "OK";
        } else if (!drive_ok && !steer_fresh) {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            st.message = "drive disconnected; steer feedback stale or absent";
        } else if (!drive_ok) {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = "drive Arduino disconnected";
        } else {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = "SPARK MAX feedback stale or absent";
        }

        st.values.push_back(kv("drive_connected",      drive_ok));
        st.values.push_back(kv("drive_open_loop",      true));
        st.values.push_back(kv("steer_feedback_fresh", steer_fresh));
        st.values.push_back(kv("steer_feedback_age_s", steer_pos_age));
        st.values.push_back(kv("steer_position_rad",   state.fb_steer_position_rad));
        st.values.push_back(kv("steer_velocity_rad_s", state.fb_steer_velocity_rad_s));
        st.values.push_back(kv("cmd_recent",           cmd_recent));
        st.values.push_back(kv("cmd_steer_rad",        state.cmd_steer_position_rad));
        st.values.push_back(kv("cmd_drive_rad_s",      state.cmd_drive_velocity_rad_s));
        st.values.push_back(kv("spark_can_id",         cfg.spark_can_id));

        msg.status.push_back(st);
    }

    // ── Per-SPARK-MAX status ──
    for (const auto & module : modules_) {
        const auto & cfg = module.config;
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name        = "warrior_swerve_driver: spark " + std::to_string(cfg.spark_can_id)
                         + " (" + cfg.name + ")";
        const std::string port = registry_->spark_port(cfg.spark_can_id);
        const bool connected = !port.empty();
        st.hardware_id = connected ? port : "(not connected)";
        if (!connected) {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            st.message = "SPARK MAX not on USB";
        } else {
            const auto applied = registry_->spark_applied_pct(cfg.spark_can_id);
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
            st.message = "connected on " + port;
            st.values.push_back(kv("applied_pct", applied.value_or(0.0f)));
        }
        st.values.push_back(kv("connected", connected));
        st.values.push_back(kv("port", connected ? port : std::string("(none)")));
        st.values.push_back(kv("can_id", cfg.spark_can_id));
        msg.status.push_back(st);
    }

    diag_pub_->publish(msg);
}

}  // namespace warrior::driver
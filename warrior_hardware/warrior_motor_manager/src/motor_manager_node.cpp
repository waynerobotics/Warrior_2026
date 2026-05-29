#include "warrior_motor_manager/motor_manager_node.hpp"

#include <chrono>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "warrior_motor_manager/serial_protocol.hpp"
#include "warrior_motor_manager/sparkmax_frame.hpp"
#include "warrior_motor_manager/unit_conversions.hpp"

namespace warrior::hardware {

namespace {

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

MotorManagerNode::MotorManagerNode() : rclcpp::Node("warrior_motor_manager")
{
    command_topic_       = declare_parameter<std::string>("command_topic", "/warrior_swerve_command");
    state_topic_         = declare_parameter<std::string>("state_topic",   "/warrior_swerve_state");
    update_rate_hz_      = declare_parameter<double>("update_rate_hz", 50.0);
    command_timeout_s_   = declare_parameter<double>("command_timeout_s", 0.5);
    steer_stale_after_s_ = declare_parameter<double>("steer_stale_after_s", 0.5);
    discovery_period_s_  = declare_parameter<double>("discovery_period_s", 2.0);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 1.0);
    baud_rate_           = declare_parameter<int>("arduino.baud_rate", 115200);

    // Declared but unused — kept so existing YAML configs don't generate
    // "parameter not declared" warnings while folks migrate.
    (void) declare_parameter<std::string>("sparkmax.slcan_interface", "auto");
    (void) declare_parameter<std::string>("sparkmax.bitrate_code", "8");

    load_modules();

    DeviceRegistry::ArduinoConfig arduino_cfg;
    arduino_cfg.baud_rate = baud_rate_;
    arduino_cfg.wanted_names.reserve(modules_.size());
    for (const auto & m : modules_) {
        if (!m.config.drive_device_name.empty()) {
            arduino_cfg.wanted_names.push_back(m.config.drive_device_name);
        }
    }

    DeviceRegistry::SparkConfig spark_cfg;
    spark_cfg.wanted_can_ids.reserve(modules_.size());
    for (const auto & m : modules_) {
        if (m.config.spark_can_id > 0) {
            spark_cfg.wanted_can_ids.push_back(m.config.spark_can_id);
        }
    }

    registry_ = std::make_unique<DeviceRegistry>(
        get_logger(), std::move(arduino_cfg), std::move(spark_cfg),
        std::chrono::duration<double>(discovery_period_s_));
    registry_->start();

    cmd_sub_ = create_subscription<warrior_msgs::msg::SwerveCmd>(
        command_topic_, rclcpp::QoS(10),
        std::bind(&MotorManagerNode::on_command, this, std::placeholders::_1));

    state_pub_ = create_publisher<warrior_msgs::msg::SwerveState>(
        state_topic_, rclcpp::QoS(10));
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10));

    const auto update_period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    update_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(update_period),
        std::bind(&MotorManagerNode::update, this));

    const auto diag_period = std::chrono::duration<double>(1.0 / diagnostics_rate_hz_);
    diag_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(diag_period),
        std::bind(&MotorManagerNode::publish_diagnostics, this));

    RCLCPP_INFO(get_logger(),
        "warrior_motor_manager up: %zu modules, sub=%s, pub=%s, rate=%.1f Hz, "
        "timeout=%.2fs, steer_stale=%.2fs, baud=%d, scan=%.1fs, diag=%.1f Hz",
        modules_.size(), command_topic_.c_str(), state_topic_.c_str(),
        update_rate_hz_, command_timeout_s_, steer_stale_after_s_, baud_rate_,
        discovery_period_s_, diagnostics_rate_hz_);
}

MotorManagerNode::~MotorManagerNode()
{
    if (registry_) {
        registry_->stop();
    }
}

void MotorManagerNode::send_safe_stop()
{
    if (registry_) {
        RCLCPP_INFO(get_logger(), "shutdown: sending drive 0%% to all connected Arduinos");
        registry_->send_zero_to_all_arduinos();
    }
}

void MotorManagerNode::load_modules()
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
        ModuleConfig cfg;
        cfg.name                            = name;
        cfg.drive_device_name               = declare_parameter<std::string>("modules." + name + ".drive_device_name", "");
        cfg.steer_device_name               = declare_parameter<std::string>("modules." + name + ".steer_device_name", "");
        cfg.spark_can_id                    = declare_parameter<int>(        "modules." + name + ".spark_can_id", 0);
        cfg.steer_motor_rot_per_module_rot  = declare_parameter<double>(     "modules." + name + ".steer_motor_rot_per_module_rot", 1.0);
        cfg.steer_offset_rad                = declare_parameter<double>(     "modules." + name + ".steer_offset_rad", 0.0);
        cfg.steer_sign                      = declare_parameter<double>(     "modules." + name + ".steer_sign", 1.0);
        cfg.drive_sign                      = declare_parameter<double>(     "modules." + name + ".drive_sign", 1.0);
        cfg.max_drive_rad_s                 = declare_parameter<double>(     "modules." + name + ".max_drive_rad_s", 1.0);

        module_index_[name] = modules_.size();
        Module m;
        m.config = cfg;
        m.runtime.last_command_time = this->now();
        modules_.push_back(std::move(m));

        RCLCPP_INFO(get_logger(),
            "module '%s': drive=%s, steer=%s (CAN %d), "
            "gear=%.3f, offset=%.3f rad, signs=(s%+.0f,d%+.0f), max_drive=%.2f rad/s",
            cfg.name.c_str(), cfg.drive_device_name.c_str(), cfg.steer_device_name.c_str(),
            cfg.spark_can_id, cfg.steer_motor_rot_per_module_rot, cfg.steer_offset_rad,
            cfg.steer_sign, cfg.drive_sign, cfg.max_drive_rad_s);
    }
}

MotorManagerNode::Module * MotorManagerNode::find_module(const std::string & name)
{
    auto it = module_index_.find(name);
    if (it == module_index_.end()) return nullptr;
    return &modules_[it->second];
}

void MotorManagerNode::on_command(const warrior_msgs::msg::SwerveCmd::SharedPtr msg)
{
    Module * module = find_module(msg->swerve_id);
    if (!module) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Ignoring command for unknown swerve_id '%s'", msg->swerve_id.c_str());
        return;
    }
    module->runtime.cmd_steer_position_rad   = msg->steer_position_rad;
    module->runtime.cmd_drive_velocity_rad_s = msg->drive_velocity_rad_s;
    module->runtime.have_command             = true;
    module->runtime.last_command_time        = this->now();
}

void MotorManagerNode::drain_and_log_arduino_messages()
{
    if (!registry_) return;
    for (const auto & [device_name, line] : registry_->drain_arduino_messages()) {
        const auto fields = serial_protocol::parse_frame(line);
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

void MotorManagerNode::update()
{
    const auto now = this->now();

    drain_and_log_arduino_messages();

    for (auto & module : modules_) {
        const auto & cfg = module.config;
        auto & rt = module.runtime;

        const bool timed_out = rt.have_command &&
            (now - rt.last_command_time).seconds() > command_timeout_s_;

        const double steer_cmd = rt.have_command ? rt.cmd_steer_position_rad : 0.0;
        const double drive_cmd = (rt.have_command && !timed_out) ? rt.cmd_drive_velocity_rad_s : 0.0;

        const double motor_rotations = steer_rad_to_motor_rotations(steer_cmd, cfg);
        const int    drive_percent   = drive_rad_s_to_percent(drive_cmd, cfg);

        // ── Drive path (Arduino) ─────────────────────────────────────────
        const bool drive_connected =
            registry_ && registry_->is_arduino_connected(cfg.drive_device_name);

        std::string drive_status;
        if (!drive_connected) {
            drive_status = "scanning";
        } else if (timed_out) {
            drive_status = "timeout";
            registry_->send_drive_percent(cfg.drive_device_name, 0);
        } else if (rt.have_command) {
            const bool ok = registry_->send_drive_percent(cfg.drive_device_name, drive_percent);
            drive_status = ok ? "active" : "write_failed";
        } else {
            drive_status = "idle";
            registry_->send_drive_percent(cfg.drive_device_name, 0);
        }

        // ── Steer path (SPARK MAX over per-USB SLCAN) ────────────────────
        const bool spark_connected =
            registry_ && registry_->is_spark_connected(cfg.spark_can_id);

        // Pull latest position from the session — updates rt.fb_steer_*
        // and rt.last_steer_pos_time from registry state.
        if (spark_connected) {
            const auto pos_rot = registry_->spark_position_rot(cfg.spark_can_id);
            const double pos_age =
                registry_->seconds_since_spark_position(cfg.spark_can_id, now);
            if (pos_rot.has_value() && pos_age >= 0.0
                && pos_age < steer_stale_after_s_)
            {
                rt.fb_steer_position_rad =
                    motor_rotations_to_steer_rad(static_cast<double>(*pos_rot), cfg);
                rt.last_steer_pos_time = now - rclcpp::Duration::from_seconds(pos_age);
            }
        }

        const bool steer_pos_fresh =
            rt.last_steer_pos_time.nanoseconds() > 0 &&
            (now - rt.last_steer_pos_time).seconds() < steer_stale_after_s_;

        std::string steer_status;
        bool steer_connected_now = false;
        if (!spark_connected) {
            steer_status = "scanning";
        } else if (rt.have_command && !timed_out) {
            const bool ok = registry_->send_steer_position(
                cfg.spark_can_id, static_cast<float>(motor_rotations));
            steer_connected_now = ok && steer_pos_fresh;
            if (!ok) {
                steer_status = "write_failed";
            } else if (!steer_pos_fresh) {
                steer_status = "no_feedback";
            } else {
                steer_status = "active";
            }
        } else {
            // No command (or command timed out) — stop driving setpoints but
            // keep the session's heartbeat going so status frames flow.
            registry_->release_steer(cfg.spark_can_id);
            steer_status = steer_pos_fresh ? "idle" : "no_feedback";
            steer_connected_now = steer_pos_fresh;
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[%s] steer cmd=%.3f rad / fb=%.3f rad (%s) | drive cmd=%.3f rad/s -> %d%% (%s)",
            cfg.name.c_str(),
            steer_cmd, rt.fb_steer_position_rad, steer_status.c_str(),
            drive_cmd, drive_percent, drive_status.c_str());

        warrior_msgs::msg::SwerveState state_msg;
        state_msg.swerve_id              = cfg.name;
        state_msg.steer_position_rad     = rt.fb_steer_position_rad;
        state_msg.steer_velocity_rad_s   = rt.fb_steer_velocity_rad_s;

        // Drive velocity has no encoder feedback in this hardware path; echo the
        // commanded value. Consumers should treat this as open-loop.
        state_msg.drive_position_rad     = 0.0;
        state_msg.drive_velocity_rad_s   = drive_cmd;
        state_msg.steer_connected        = steer_connected_now;
        state_msg.drive_connected        = drive_connected;
        state_msg.steer_status           = steer_status;
        state_msg.drive_status           = drive_status;
        state_msg.stamp                  = now;
        state_pub_->publish(state_msg);
    }
}

void MotorManagerNode::publish_diagnostics()
{
    if (!registry_) return;
    const auto now = this->now();

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now;

    for (const auto & module : modules_) {
        const auto & cfg = module.config;
        const auto & rt  = module.runtime;

        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name        = "warrior_motor_manager: " + cfg.name;
        st.hardware_id = cfg.drive_device_name + " / spark can=" + std::to_string(cfg.spark_can_id);

        const bool drive_ok = registry_->is_arduino_connected(cfg.drive_device_name);
        const double steer_pos_age = rt.last_steer_pos_time.nanoseconds() > 0
            ? (now - rt.last_steer_pos_time).seconds() : -1.0;
        const bool steer_fresh = steer_pos_age >= 0.0 && steer_pos_age < steer_stale_after_s_;
        const bool cmd_recent  = rt.have_command &&
                                 (now - rt.last_command_time).seconds() < command_timeout_s_;

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

        st.values.push_back(kv("drive_connected",          drive_ok));
        st.values.push_back(kv("drive_open_loop",          true));
        st.values.push_back(kv("steer_feedback_fresh",     steer_fresh));
        st.values.push_back(kv("steer_feedback_age_s",     steer_pos_age));
        st.values.push_back(kv("steer_position_rad",       rt.fb_steer_position_rad));
        st.values.push_back(kv("steer_velocity_rad_s",     rt.fb_steer_velocity_rad_s));
        st.values.push_back(kv("cmd_recent",               cmd_recent));
        st.values.push_back(kv("cmd_steer_rad",            rt.cmd_steer_position_rad));
        st.values.push_back(kv("cmd_drive_rad_s",          rt.cmd_drive_velocity_rad_s));
        st.values.push_back(kv("spark_can_id",             cfg.spark_can_id));

        msg.status.push_back(st);
    }

    for (const auto & module : modules_) {
        const auto & cfg = module.config;
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name        = "warrior_motor_manager: spark " + std::to_string(cfg.spark_can_id)
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

}  // namespace warrior::hardware

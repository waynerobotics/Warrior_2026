#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "warrior_driver/swerve/steer_calibration_node.hpp"
#include "warrior_driver/swerve/unit_conversions.hpp"

namespace warrior::driver {

// ─────────────────────────────────────────────────────────────────────────────
SteerCalibrationNode::SteerCalibrationNode()
: rclcpp::Node("steer_calibration")
{
    state_topic_        = declare_parameter<std::string>("state_topic",        "/warrior_swerve_state");
    feedback_timeout_s_ = declare_parameter<double>     ("feedback_timeout_s", 2.0);
    output_dir_         = declare_parameter<std::string>("output_dir",         "/tmp");

    load_modules();

    state_sub_ = create_subscription<warrior_msgs::msg::SwerveState>(
        state_topic_, rclcpp::QoS(10),
        std::bind(&SteerCalibrationNode::on_state, this, std::placeholders::_1));

    calibrate_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/calibrate",
        std::bind(&SteerCalibrationNode::on_calibrate, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
        "steer_calibration node ready with %zu module(s). "
        "Call service '%s' after physically aligning wheels straight forward.",
        modules_.size(), (get_name() + std::string("/calibrate")).c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
void SteerCalibrationNode::load_modules()
{
    const auto names = declare_parameter<std::vector<std::string>>(
        "module_names", std::vector<std::string>{});

    if (names.empty()) {
        RCLCPP_ERROR(get_logger(),
            "Parameter 'module_names' is empty — did you load warrior_driver.yaml?");
        return;
    }

    modules_.reserve(names.size());
    for (const auto & name : names) {
        SwerveModuleConfig cfg;
        cfg.name                           = name;
        cfg.spark_can_id                   = declare_parameter<int>   ("modules." + name + ".spark_can_id",                    0);
        cfg.steer_motor_rot_per_module_rot = declare_parameter<double>("modules." + name + ".steer_motor_rot_per_module_rot",  1.0);
        cfg.steer_offset_rad               = declare_parameter<double>("modules." + name + ".steer_offset_rad",                0.0);
        cfg.steer_sign                     = declare_parameter<double>("modules." + name + ".steer_sign",                      1.0);
        // drive parameters not needed for steering calibration, but declared
        // so that sharing the same YAML as warrior_driver doesn't raise warnings
        cfg.drive_device_name  = declare_parameter<std::string>("modules." + name + ".drive_device_name", "");
        cfg.steer_device_name  = declare_parameter<std::string>("modules." + name + ".steer_device_name", "");
        cfg.drive_sign         = declare_parameter<double>     ("modules." + name + ".drive_sign",         1.0);
        cfg.max_drive_rad_s    = declare_parameter<double>     ("modules." + name + ".max_drive_rad_s",    1.0);

        module_index_[name] = modules_.size();
        CalibModule m;
        m.config = cfg;
        m.state.last_feedback_time = this->now();
        modules_.push_back(std::move(m));

        RCLCPP_INFO(get_logger(),
            "  module '%s': spark_can_id=%d, gear=%.1f, "
            "current_offset=%.4f rad, steer_sign=%.0f",
            cfg.name.c_str(), cfg.spark_can_id,
            cfg.steer_motor_rot_per_module_rot, cfg.steer_offset_rad, cfg.steer_sign);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void SteerCalibrationNode::on_state(const warrior_msgs::msg::SwerveState::SharedPtr msg)
{
    auto it = module_index_.find(msg->swerve_id);
    if (it == module_index_.end()) return;

    auto & mod         = modules_[it->second];
    mod.state.steer_position_rad = msg->steer_position_rad;
    mod.state.have_feedback      = msg->steer_connected;
    mod.state.last_feedback_time = rclcpp::Time(msg->stamp);
}

// ─────────────────────────────────────────────────────────────────────────────
void SteerCalibrationNode::on_calibrate(
    const std_srvs::srv::Trigger::Request::SharedPtr  /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr        response)
{
    const auto now = this->now();

    RCLCPP_INFO(get_logger(), "=== Steer calibration triggered ===");

    // ── Validate freshness ────────────────────────────────────────────────────
    std::vector<std::string> stale_modules;
    for (const auto & mod : modules_) {
        const double age = (now - mod.state.last_feedback_time).seconds();
        if (!mod.state.have_feedback || age > feedback_timeout_s_) {
            stale_modules.push_back(mod.config.name +
                " (age=" + std::to_string(age) + "s, connected=" +
                (mod.state.have_feedback ? "true" : "false") + ")");
        }
    }

    if (!stale_modules.empty()) {
        std::string err = "Calibration aborted — stale/missing feedback for: ";
        for (const auto & s : stale_modules) err += "\n  " + s;
        RCLCPP_ERROR(get_logger(), "%s", err.c_str());
        response->success = false;
        response->message = err;
        return;
    }

    // ── Compute new offsets ───────────────────────────────────────────────────
    //
    // The unit_conversions formula is:
    //   steer_rad = (motor_rotations / gear * 2π - steer_offset_rad) / steer_sign
    //
    // We want the new offset that makes the *current* encoder position read as
    // 0 rad (straight forward):
    //   0 = (motor_rotations / gear * 2π - new_offset) / sign
    //   new_offset = motor_rotations / gear * 2π
    //
    // We don't have raw motor_rotations here — only the already-decoded
    // steer_position_rad.  Working backwards:
    //   motor_rotations / gear * 2π = steer_rad * sign + old_offset
    //   => new_offset = current_steer_rad * steer_sign + old_steer_offset_rad
    //
    struct Result {
        std::string name;
        double      old_offset_rad;
        double      captured_steer_rad;
        double      new_offset_rad;
        int         spark_can_id;
    };
    std::vector<Result> results;
    results.reserve(modules_.size());

    for (const auto & mod : modules_) {
        const auto & cfg = mod.config;
        const double captured  = mod.state.steer_position_rad;
        const double new_off   = captured * cfg.steer_sign + cfg.steer_offset_rad;

        results.push_back({
            cfg.name,
            cfg.steer_offset_rad,
            captured,
            new_off,
            cfg.spark_can_id
        });

        RCLCPP_INFO(get_logger(),
            "  [%s] captured=%.4f rad  old_offset=%.4f rad  "
            "=> new_offset=%.4f rad  (spark CAN %d)",
            cfg.name.c_str(), captured,
            cfg.steer_offset_rad, new_off, cfg.spark_can_id);
    }

    // ── Build YAML snippet ────────────────────────────────────────────────────
    std::ostringstream yaml;
    yaml << "# Steer calibration result — generated by steer_calibration_node\n";
    yaml << "# Copy the steer_offset_rad values below into warrior_driver.yaml\n";
    yaml << "#\n";
    yaml << "# Procedure: wheels were physically aligned STRAIGHT FORWARD,\n";
    yaml << "# then ~/calibrate was called.\n";
    yaml << "#\n";

    // Timestamp
    std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char tbuf[32];
    std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
    yaml << "# Timestamp: " << tbuf << "\n\n";

    yaml << "warrior_driver:\n";
    yaml << "  ros__parameters:\n";
    yaml << "    modules:\n";
    for (const auto & r : results) {
        yaml << "      " << r.name << ":\n";
        yaml << "        steer_offset_rad: " << r.new_offset_rad << "  "
             << "# was " << r.old_offset_rad << "\n";
    }

    const std::string yaml_str = yaml.str();

    // Print to console so the operator can copy it immediately
    RCLCPP_INFO(get_logger(), "\n\n%s", yaml_str.c_str());

    // ── Write to file ─────────────────────────────────────────────────────────
    std::string filename = output_dir_ + "/steer_calibration_" +
        std::string(tbuf) + ".yaml";
    // Replace ':' so the filename is portable
    for (auto & c : filename) if (c == ':') c = '-';

    try {
        std::filesystem::create_directories(output_dir_);
        std::ofstream f(filename);
        if (!f) throw std::runtime_error("could not open " + filename);
        f << yaml_str;
        f.close();
        RCLCPP_INFO(get_logger(), "Calibration results written to: %s", filename.c_str());
    } catch (const std::exception & ex) {
        RCLCPP_WARN(get_logger(), "Could not write calibration file: %s", ex.what());
    }

    response->success = true;
    response->message = "Calibration complete. Results written to " + filename +
                        "\nPaste the YAML snippet into warrior_driver.yaml and rebuild.";
}

}  // namespace warrior::driver

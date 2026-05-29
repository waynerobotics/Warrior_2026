// steer_calibration_node.cpp
//
// Standalone steer-offset calibrator for the Warrior swerve drive.
//
// Opens each SPARK MAX USB-SLCAN port directly and sends ONLY the periodic
// heartbeat frames (mode-bitmask + enable).  NO setpoint frames are ever
// sent, so the SPARK MAX streams its encoder position but never drives.
//
// The wheel positions must be set BEFORE running this node:
//   1. Power off robot.
//   2. Manually rotate every wheel straight forward.
//   3. Power on SPARK MAXes (they boot neutral — wheels stay free).
//   4. Start this node, then call ~/calibrate.
//
// See sparkmax_frame.hpp for the CAN frame layout details.

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <thread>

#include "warrior_driver/swerve/steer_calibration_node.hpp"
#include "warrior_driver/swerve/unit_conversions.hpp"
#include "warrior_driver/sparkmax/sparkmax_frame.hpp"
#include "warrior_driver/sparkmax/sparkmax_session.hpp"  // for list_sparkmax_ports()

namespace warrior::driver {

// ─────────────────────────────────────────────────────────────────────────────
SteerCalibrationNode::SteerCalibrationNode()
: rclcpp::Node("steer_calibration")
{
    bitrate_code_   = static_cast<char>(
        declare_parameter<int>   ("bitrate_code",   8) + '0');
    read_timeout_s_ = declare_parameter<double>     ("read_timeout_s",  5.0);
    output_dir_     = declare_parameter<std::string>("output_dir",       "/tmp/warrior_calibration");

    load_modules();

    calibrate_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/calibrate",
        std::bind(&SteerCalibrationNode::on_calibrate, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
        "steer_calibration node ready (%zu module(s)).\n"
        "  Ensure warrior_driver is NOT running, wheels are straight forward,\n"
        "  then call: ros2 service call /steer_calibration/calibrate std_srvs/srv/Trigger \"{}\"",
        modules_.size());
}

// ─────────────────────────────────────────────────────────────────────────────
void SteerCalibrationNode::load_modules()
{
    const auto names = declare_parameter<std::vector<std::string>>(
        "module_names", std::vector<std::string>{});

    if (names.empty()) {
        RCLCPP_ERROR(get_logger(), "Parameter 'module_names' is empty — check steer_calibration.yaml");
        return;
    }

    modules_.reserve(names.size());
    for (const auto & name : names) {
        SwerveModuleConfig cfg;
        cfg.name                           = name;
        cfg.spark_can_id                   = declare_parameter<int>   ("modules." + name + ".spark_can_id",                   0);
        cfg.steer_motor_rot_per_module_rot = declare_parameter<double>("modules." + name + ".steer_motor_rot_per_module_rot", 1.0);
        cfg.steer_offset_rad               = declare_parameter<double>("modules." + name + ".steer_offset_rad",               0.0);
        cfg.steer_sign                     = declare_parameter<double>("modules." + name + ".steer_sign",                     1.0);
        // Declare but ignore drive params so this node can share the same YAML
        // as warrior_driver without "undeclared parameter" warnings.
        cfg.drive_device_name = declare_parameter<std::string>("modules." + name + ".drive_device_name", "");
        cfg.steer_device_name = declare_parameter<std::string>("modules." + name + ".steer_device_name", "");
        cfg.drive_sign        = declare_parameter<double>     ("modules." + name + ".drive_sign",         1.0);
        cfg.max_drive_rad_s   = declare_parameter<double>     ("modules." + name + ".max_drive_rad_s",    1.0);

        can_id_to_module_[cfg.spark_can_id] = modules_.size();
        CalibModule m;
        m.config = cfg;
        modules_.push_back(std::move(m));

        RCLCPP_INFO(get_logger(),
            "  module '%s': spark_can_id=%d, gear=%.1f, "
            "current_offset=%.4f rad, steer_sign=%.0f",
            cfg.name.c_str(), cfg.spark_can_id,
            cfg.steer_motor_rot_per_module_rot,
            cfg.steer_offset_rad, cfg.steer_sign);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SLCAN port helpers
// ─────────────────────────────────────────────────────────────────────────────

bool SteerCalibrationNode::open_slcan_port(PortState & ps) const
{
    ps.fd = ::open(ps.path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (ps.fd < 0) {
        RCLCPP_WARN(get_logger(), "  open(%s) failed: %s", ps.path.c_str(), strerror(errno));
        return false;
    }

    if (ioctl(ps.fd, TIOCEXCL) != 0) {
        RCLCPP_WARN(get_logger(), "  TIOCEXCL(%s) failed: %s", ps.path.c_str(), strerror(errno));
        ::close(ps.fd); ps.fd = -1;
        return false;
    }

    // Switch to blocking I/O
    int flags = fcntl(ps.fd, F_GETFL, 0);
    fcntl(ps.fd, F_SETFL, flags & ~O_NONBLOCK);

    // 115200 8N1 raw
    termios tio{};
    if (tcgetattr(ps.fd, &tio) != 0) { ::close(ps.fd); ps.fd = -1; return false; }
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD | CS8);
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(ps.fd, TCSANOW, &tio) != 0) { ::close(ps.fd); ps.fd = -1; return false; }
    tcflush(ps.fd, TCIOFLUSH);

    // Assert DTR + RTS
    int modem = TIOCM_DTR | TIOCM_RTS;
    ioctl(ps.fd, TIOCMBIS, &modem);

    // Open SLCAN channel: close (no-op) → set bitrate → open.
    const std::string init = std::string("C\r") + "S" + bitrate_code_ + "\r" + "O\r";
    if (!write_port(const_cast<PortState &>(ps), init)) {
        RCLCPP_WARN(get_logger(), "  SLCAN init write failed on %s", ps.path.c_str());
        ::close(ps.fd); ps.fd = -1;
        return false;
    }

    RCLCPP_INFO(get_logger(), "  opened %s", ps.path.c_str());
    return true;
}

void SteerCalibrationNode::close_slcan_port(PortState & ps) const
{
    if (ps.fd >= 0) {
        write_port(const_cast<PortState &>(ps), "C\r");  // best-effort channel close
        ::close(ps.fd);
        ps.fd = -1;
    }
}

bool SteerCalibrationNode::write_port(PortState & ps, const std::string & data) const
{
    if (ps.fd < 0) return false;
    const char * d = data.data();
    std::size_t rem = data.size();
    while (rem > 0) {
        const ssize_t n = ::write(ps.fd, d, rem);
        if (n < 0) { if (errno == EINTR) continue; return false; }
        d   += n;
        rem -= static_cast<std::size_t>(n);
    }
    return true;
}

void SteerCalibrationNode::drain_port(PortState & ps) const
{
    if (ps.fd < 0) return;

    char buf[512];
    while (true) {
        const ssize_t n = ::read(ps.fd, buf, sizeof(buf));
        if (n > 0) ps.rx_buf.append(buf, static_cast<std::size_t>(n));
        else break;
    }

    std::size_t start = 0;
    for (std::size_t i = 0; i < ps.rx_buf.size(); ++i) {
        if (ps.rx_buf[i] == '\r' || ps.rx_buf[i] == '\n') {
            if (i > start) {
                const std::string line = ps.rx_buf.substr(start, i - start);
                auto frame = sparkmax::parse_slcan_extended_rx(line);
                if (frame) {
                    const uint32_t arb = frame->arbitration_id;

                    // Learn CAN ID from the first inbound Status frame.
                    if (ps.can_id < 0) {
                        ps.can_id = static_cast<int>(arb & 0x3F);
                        RCLCPP_INFO(get_logger(),
                            "    %s → discovered CAN ID %d",
                            ps.path.c_str(), ps.can_id);
                    }

                    // Status 2 carries motor position (float32 at byte offset 4).
                    auto status = sparkmax::identify_periodic_status(arb);
                    if (status &&
                        status->status_index == sparkmax::STATUS_INDEX_POSITION &&
                        frame->dlc >= 8)
                    {
                        ps.position_rot = sparkmax::decode_status_position_rotations(
                            frame->data.data());
                    }
                }
            }
            start = i + 1;
        }
    }
    ps.rx_buf.erase(0, start);
}

// ─────────────────────────────────────────────────────────────────────────────
// Calibration service
// ─────────────────────────────────────────────────────────────────────────────

void SteerCalibrationNode::on_calibrate(
    const std_srvs::srv::Trigger::Request::SharedPtr  /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr        response)
{
    RCLCPP_INFO(get_logger(), "=== Steer calibration triggered ===");

    if (modules_.empty()) {
        response->success = false;
        response->message = "No modules configured — check steer_calibration.yaml.";
        return;
    }

    // ── 1. Discover SPARK MAX ports ───────────────────────────────────────────
    const auto port_paths = list_sparkmax_ports();
    if (port_paths.empty()) {
        response->success = false;
        response->message = "No SPARK MAX USB ports found. "
                            "Are the controllers powered and plugged in?";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "Found %zu SPARK MAX port(s):", port_paths.size());
    for (const auto & p : port_paths) RCLCPP_INFO(get_logger(), "  %s", p.c_str());

    // ── 2. Open all ports ─────────────────────────────────────────────────────
    std::vector<PortState> ports(port_paths.size());
    for (std::size_t i = 0; i < port_paths.size(); ++i) {
        ports[i].path = port_paths[i];
        open_slcan_port(ports[i]);
    }

    // Brief pause so the CAN channel is fully open before we start reading.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ── 3. Poll loop — heartbeat only, NO setpoints ───────────────────────────
    //
    // Every ~20 ms tick we send to every open port:
    //   make_mode_frame(0xFF)   ← broadcast heartbeat, keeps status streaming
    //   ENABLE_FRAME            ← FRC-style enable (broadcast)
    //
    // Once a port's CAN ID is learned from an inbound Status 0 frame, we
    // send make_enable_telemetry_frame(can_id) once to trigger Status 2.
    //
    // NO position setpoint frame is ever sent.  The SPARK MAX stays in
    // "enabled-idle" state: it holds 0% output and the wheel remains free.

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(read_timeout_s_);

    RCLCPP_INFO(get_logger(),
        "Polling for Status 2 frames (timeout %.1f s) ...", read_timeout_s_);

    while (std::chrono::steady_clock::now() < deadline) {
        // Early-exit when all configured modules have a position reading.
        std::size_t done = 0;
        for (const auto & mod : modules_) {
            for (const auto & ps : ports) {
                if (ps.can_id == mod.config.spark_can_id &&
                    !std::isnan(ps.position_rot)) { ++done; break; }
            }
        }
        if (done == modules_.size()) break;

        const std::string heartbeat =
            sparkmax::make_mode_frame(0xFF) +
            std::string(sparkmax::ENABLE_FRAME);

        for (auto & ps : ports) {
            if (ps.fd < 0) continue;

            write_port(ps, heartbeat);

            // Once we know the CAN ID, poke telemetry-enable once so the
            // controller starts emitting Status 2 (position).
            if (ps.can_id >= 0 && !ps.telemetry_sent) {
                write_port(ps, sparkmax::make_enable_telemetry_frame(
                    static_cast<uint32_t>(ps.can_id)));
                ps.telemetry_sent = true;
                RCLCPP_INFO(get_logger(),
                    "    sent telemetry-enable to CAN ID %d on %s",
                    ps.can_id, ps.path.c_str());
            }

            drain_port(ps);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // ── 4. Close all ports ────────────────────────────────────────────────────
    for (auto & ps : ports) close_slcan_port(ps);

    // ── 5. Match ports → modules ──────────────────────────────────────────────
    struct Result {
        std::string name;
        int         spark_can_id;
        double      old_offset_rad;
        double      motor_rotations;
        double      new_offset_rad;
    };
    std::vector<Result> results;
    std::vector<std::string> missing;

    for (const auto & mod : modules_) {
        const auto & cfg = mod.config;
        const PortState * found = nullptr;
        for (const auto & ps : ports) {
            if (ps.can_id == cfg.spark_can_id) { found = &ps; break; }
        }

        if (!found || std::isnan(found->position_rot)) {
            missing.push_back(cfg.name +
                " (CAN ID " + std::to_string(cfg.spark_can_id) + ")");
            continue;
        }

        // new_offset_rad = motor_rotations / gear * 2π
        //
        // Derivation: the driver decode formula is
        //   steer_rad = (motor_rot / gear * 2π - offset) / sign
        // Setting steer_rad = 0 (straight forward) gives:
        //   offset_new = motor_rot / gear * 2π
        const double gear = cfg.steer_motor_rot_per_module_rot != 0.0
                              ? cfg.steer_motor_rot_per_module_rot : 1.0;
        const double new_off = static_cast<double>(found->position_rot) / gear * 2.0 * M_PI;

        results.push_back({cfg.name, cfg.spark_can_id,
                           cfg.steer_offset_rad,
                           static_cast<double>(found->position_rot),
                           new_off});

        RCLCPP_INFO(get_logger(),
            "  [%s] motor_rot=%.4f  old_offset=%.4f rad  => new_offset=%.6f rad  (CAN ID %d)",
            cfg.name.c_str(), found->position_rot,
            cfg.steer_offset_rad, new_off, cfg.spark_can_id);
    }

    if (!missing.empty()) {
        std::string err = "Calibration incomplete — no Status 2 received for:";
        for (const auto & s : missing) err += "\n  " + s;
        err += "\nCheck: (1) warrior_driver is NOT running, "
               "(2) SPARK MAXes are powered, (3) USB cables are connected.";
        RCLCPP_ERROR(get_logger(), "%s", err.c_str());
        response->success = false;
        response->message = err;
        return;
    }

    // ── 6. Build YAML snippet ─────────────────────────────────────────────────
    std::time_t t = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());
    char tbuf[32];
    std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));

    std::ostringstream yaml;
    yaml << "# Steer calibration result — generated by steer_calibration_node\n"
         << "# Timestamp: " << tbuf << "\n"
         << "#\n"
         << "# Procedure: SPARK MAXes were powered on with wheels physically\n"
         << "# aligned STRAIGHT FORWARD (warrior_driver was NOT running).\n"
         << "# Motor positions were read passively from Status 2 frames.\n"
         << "#\n"
         << "# Paste these steer_offset_rad values into warrior_driver.yaml,\n"
         << "# then rebuild: colcon build --packages-select warrior_driver\n\n"
         << "warrior_driver:\n"
         << "  ros__parameters:\n"
         << "    modules:\n";
    for (const auto & r : results) {
        yaml << "      " << r.name << ":\n"
             << "        steer_offset_rad: " << r.new_offset_rad
             << "  # was " << r.old_offset_rad
             << "  (motor_rot=" << r.motor_rotations << ")\n";
    }

    const std::string yaml_str = yaml.str();
    RCLCPP_INFO(get_logger(), "\n\n%s", yaml_str.c_str());

    // ── 7. Write to file ──────────────────────────────────────────────────────
    std::string filename = output_dir_ + "/steer_calibration_" + tbuf + ".yaml";
    for (auto & c : filename) if (c == ':') c = '-';

    try {
        std::filesystem::create_directories(output_dir_);
        std::ofstream f(filename);
        if (!f) throw std::runtime_error("could not open " + filename);
        f << yaml_str;
        RCLCPP_INFO(get_logger(), "Results written to: %s", filename.c_str());
    } catch (const std::exception & ex) {
        RCLCPP_WARN(get_logger(), "Could not write file: %s", ex.what());
    }

    response->success = true;
    response->message =
        std::string("Calibration complete. Results written to ") + filename +
        "\nPaste the steer_offset_rad values into warrior_driver.yaml and rebuild.";
}

}  // namespace warrior::driver


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

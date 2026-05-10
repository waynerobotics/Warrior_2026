/**
 * spark_max_driver.cpp
 * --------------------
 * ROS 2 node: spark_max_driver
 *
 * Manages a single USB CDC connection to one REV SPARK MAX motor controller
 * and provides closed-loop position control over /spark_cmd.
 *
 * Architecture
 * ------------
 *  - Discovery thread: scans /dev/ttyACM* ports filtered by USB VID/PID,
 *    opens the SPARK MAX exclusively, sends an initial heartbeat.
 *  - Heartbeat thread: sends a heartbeat packet every `heartbeat_ms` ms.
 *    CRITICAL — the SPARK MAX disables motor output if no heartbeat arrives
 *    within ~100 ms.  This thread runs independently of the ROS executor.
 *  - /spark_cmd subscriber (ROS callback): writes a 12-byte position setpoint
 *    packet if msg.target matches this node's device_name parameter.
 *  - /spark_feedback publisher: emits telemetry read from periodic status
 *    frames (currently stubbed — see NOTE below).
 *
 * ROS 2 parameters
 * ----------------
 *  device_name               (string, default "spark_max")
 *      Logical name used to match SparkCommand.target.
 *  device_id                 (int,    default 1)
 *      CAN Device ID assigned in REV Hardware Client (1–62).
 *  heartbeat_ms              (int,    default 50)
 *      Heartbeat interval in milliseconds (keep well below 100 ms).
 *  discovery_retry_period_s  (double, default 2.0)
 *      Seconds between port-scan retries when no SPARK MAX is found.
 *
 * Topics subscribed
 * -----------------
 *  /spark_cmd  (warrior_msgs/SparkCommand)
 *
 * Topics published
 * ----------------
 *  /spark_feedback  (warrior_msgs/SparkFeedback)
 *
 * NOTE on telemetry
 * -----------------
 * The SPARK MAX sends periodic status frames over USB automatically.
 * Parsing those frames requires reading 12-byte packets and matching
 * the command ID to the appropriate status frame (API Class 0x06,
 * various API Indices for position/velocity/current/temperature).
 * This is stubbed for now — feedback is published with zeros until
 * the read loop is implemented.  Capture USB traffic from REV Hardware
 * Client to identify the exact frame IDs.
 */

#include <rclcpp/rclcpp.hpp>
#include <warrior_msgs/msg/spark_command.hpp>
#include <warrior_msgs/msg/spark_feedback.hpp>

#include "warrior_sparkmax/spark_protocol.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// POSIX serial
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using SparkCommand  = warrior_msgs::msg::SparkCommand;
using SparkFeedback = warrior_msgs::msg::SparkFeedback;
using namespace warrior_sparkmax;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Helpers — USB identification and serial port management
// ---------------------------------------------------------------------------

/// Read a single-line text file from sysfs (e.g. idVendor, idProduct).
static std::string read_sysfs(const std::string & path)
{
    std::ifstream f(path);
    std::string s;
    if (f) std::getline(f, s);
    return s;
}

/// Return true if /dev/ttyACMx was enumerated by a SPARK MAX (VID/PID match).
static bool is_sparkmax_port(const std::string & dev)
{
    // Walk sysfs: /sys/class/tty/ttyACM0/device/ contains idVendor, idProduct
    // (symlinked from the USB device node)
    const std::string base = std::filesystem::path(dev).filename().string();
    const std::string sys  = "/sys/class/tty/" + base + "/device/";
    try {
        const unsigned vid = std::stoul(read_sysfs(sys + "idVendor"),  nullptr, 16);
        const unsigned pid = std::stoul(read_sysfs(sys + "idProduct"), nullptr, 16);
        return vid == SPARK_USB_VID && pid == SPARK_USB_PID;
    } catch (...) {
        return false;
    }
}

/// Open port at 115200 8N1, raw mode, 100 ms read timeout.  Returns fd or -1.
static int open_serial(const std::string & port)
{
    // O_EXCL — exclusive kernel lock, prevents two processes sharing the port
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_EXCL);
    if (fd < 0) return -1;

    struct termios tty{};
    if (::tcgetattr(fd, &tty) != 0) { ::close(fd); return -1; }

    ::cfsetispeed(&tty, B115200);
    ::cfsetospeed(&tty, B115200);

    // 8N1, no flow control, raw (binary) mode
    tty.c_cflag |=  (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;   tty.c_cflag |= CS8;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_lflag  =  0;  // no canonical mode, no echo
    tty.c_oflag  =  0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;  // 100 ms inter-byte timeout

    if (::tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return -1; }
    return fd;
}

/// Write exactly SPARK_PACKET_BYTES to fd.  Returns true on success.
static bool write_packet(int fd, const SparkPacket & p)
{
    const ssize_t n = ::write(fd, p.bytes, SPARK_PACKET_BYTES);
    return n == SPARK_PACKET_BYTES;
}

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

class SparkMaxDriverNode : public rclcpp::Node
{
public:
    explicit SparkMaxDriverNode()
    : Node("spark_max_driver")
    {
        device_name_     = declare_parameter("device_name", std::string("spark_max"));
        device_id_       = static_cast<uint32_t>(declare_parameter("device_id", 1));
        heartbeat_ms_    = declare_parameter("heartbeat_ms", 50);
        scan_period_s_   = declare_parameter("discovery_retry_period_s", 2.0);

        sub_ = create_subscription<SparkCommand>(
            "/spark_cmd", 10,
            std::bind(&SparkMaxDriverNode::cmd_cb, this, std::placeholders::_1));

        pub_ = create_publisher<SparkFeedback>("/spark_feedback", 10);

        stop_.store(false);
        heartbeat_thread_ = std::thread(&SparkMaxDriverNode::heartbeat_loop, this);
        discovery_thread_ = std::thread(&SparkMaxDriverNode::discovery_loop, this);

        RCLCPP_INFO(get_logger(),
            "spark_max_driver started — device_name='%s'  device_id=%u  "
            "heartbeat=%d ms",
            device_name_.c_str(), device_id_, heartbeat_ms_);
    }

    ~SparkMaxDriverNode() override
    {
        stop_.store(true);
        if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
        if (discovery_thread_.joinable()) discovery_thread_.join();
        close_port();
    }

private:

    // ------------------------------------------------------------------
    // Port lifecycle
    // ------------------------------------------------------------------

    void close_port()
    {
        std::lock_guard<std::mutex> lk(fd_mutex_);
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
            port_.clear();
        }
    }

    // ------------------------------------------------------------------
    // Discovery — background thread
    // ------------------------------------------------------------------

    void discovery_loop()
    {
        while (!stop_.load()) {
            {
                std::lock_guard<std::mutex> lk(fd_mutex_);
                if (fd_ >= 0) {
                    // Already connected — nothing to do this iteration
                    std::this_thread::sleep_for(
                        std::chrono::duration<double>(scan_period_s_));
                    continue;
                }
            }

            scan_ports();

            std::this_thread::sleep_for(
                std::chrono::duration<double>(scan_period_s_));
        }
    }

    void scan_ports()
    {
        std::vector<std::string> candidates;
        try {
            for (const auto & entry :
                 std::filesystem::directory_iterator("/dev"))
            {
                const std::string name = entry.path().string();
                if (name.find("ttyACM") != std::string::npos)
                    candidates.push_back(name);
            }
        } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "[discovery] /dev scan error: %s", e.what());
            return;
        }

        std::sort(candidates.begin(), candidates.end());

        for (const auto & port : candidates) {
            if (stop_.load()) return;

            if (!is_sparkmax_port(port)) {
                RCLCPP_DEBUG(get_logger(),
                    "[discovery] %s — VID/PID not SPARK MAX, skipping",
                    port.c_str());
                continue;
            }

            RCLCPP_INFO(get_logger(),
                "[discovery] SPARK MAX VID/PID on %s — opening…", port.c_str());

            const int fd = open_serial(port);
            if (fd < 0) {
                RCLCPP_WARN(get_logger(),
                    "[discovery] Cannot open %s: %s", port.c_str(), strerror(errno));
                continue;
            }

            // Send initial heartbeat to wake the controller
            SparkPacket hb = SparkPacket::heartbeat();
            if (!write_packet(fd, hb)) {
                RCLCPP_WARN(get_logger(),
                    "[discovery] Initial heartbeat write failed on %s", port.c_str());
                ::close(fd);
                continue;
            }

            {
                std::lock_guard<std::mutex> lk(fd_mutex_);
                fd_   = fd;
                port_ = port;
            }

            RCLCPP_INFO(get_logger(),
                "[discovery] Connected to SPARK MAX '%s' (device_id=%u) on %s",
                device_name_.c_str(), device_id_, port.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(),
            "[discovery] No SPARK MAX found — retry in %.1f s", scan_period_s_);
    }

    // ------------------------------------------------------------------
    // Heartbeat — background thread (CRITICAL — do not block this)
    // ------------------------------------------------------------------

    void heartbeat_loop()
    {
        const auto interval = std::chrono::milliseconds(heartbeat_ms_);
        while (!stop_.load()) {
            {
                std::lock_guard<std::mutex> lk(fd_mutex_);
                if (fd_ >= 0) {
                    SparkPacket hb = SparkPacket::heartbeat();
                    if (!write_packet(fd_, hb)) {
                        RCLCPP_WARN(get_logger(),
                            "[heartbeat] Write failed on %s — disconnecting",
                            port_.c_str());
                        ::close(fd_);
                        fd_ = -1;
                        port_.clear();
                    }
                }
            }
            std::this_thread::sleep_for(interval);
        }
    }

    // ------------------------------------------------------------------
    // /spark_cmd subscriber (ROS executor thread)
    // ------------------------------------------------------------------

    void cmd_cb(const SparkCommand::SharedPtr msg)
    {
        if (msg->target != device_name_) return;

        std::lock_guard<std::mutex> lk(fd_mutex_);
        if (fd_ < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[tx] DROP SparkCommand for '%s' — not connected",
                device_name_.c_str());
            return;
        }

        SparkPacket p = SparkPacket::position_setpoint(msg->setpoint, device_id_);
        RCLCPP_DEBUG(get_logger(),
            "[tx] position setpoint %.4f rot → %s", msg->setpoint, port_.c_str());

        if (!write_packet(fd_, p)) {
            RCLCPP_WARN(get_logger(),
                "[tx] Write error on %s — disconnecting", port_.c_str());
            ::close(fd_);
            fd_ = -1;
            port_.clear();
        }
    }

    // ------------------------------------------------------------------
    // Members
    // ------------------------------------------------------------------

    std::string  device_name_;
    uint32_t     device_id_{1};
    int          heartbeat_ms_{50};
    double       scan_period_s_{2.0};

    std::mutex   fd_mutex_;
    int          fd_{-1};
    std::string  port_;

    std::atomic<bool> stop_{false};
    std::thread  heartbeat_thread_;
    std::thread  discovery_thread_;

    rclcpp::Subscription<SparkCommand>::SharedPtr  sub_;
    rclcpp::Publisher<SparkFeedback>::SharedPtr    pub_;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SparkMaxDriverNode>());
    rclcpp::shutdown();
    return 0;
}

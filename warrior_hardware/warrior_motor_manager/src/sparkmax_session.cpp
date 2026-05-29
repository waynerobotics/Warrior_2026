#include "warrior_motor_manager/sparkmax_session.hpp"

#include <fcntl.h>
#include <glob.h>
#include <limits.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <thread>

#include "warrior_motor_manager/sparkmax_frame.hpp"

namespace warrior::hardware {

namespace {

constexpr int   BAUD_RATE      = 115200;
constexpr auto  TX_PERIOD      = std::chrono::milliseconds(20);
constexpr auto  RX_IDLE_SLEEP  = std::chrono::milliseconds(5);

bool configure_115200_raw(int fd)
{
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) return false;
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE; tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB; tio.c_cflag &= ~CSTOPB; tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) != 0) return false;
    tcflush(fd, TCIOFLUSH);
    return true;
}

// Walk up from /sys/class/tty/<name>/device until idVendor + idProduct exist.
// Returns (vid, pid) or 0/0 on failure.
std::pair<unsigned, unsigned> read_usb_vid_pid_for_tty(const std::string & tty_basename)
{
    const std::string start = "/sys/class/tty/" + tty_basename + "/device";
    char real[PATH_MAX];
    if (!realpath(start.c_str(), real)) return {0u, 0u};
    std::string p = real;
    while (p.size() > 1) {
        std::ifstream vfile(p + "/idVendor");
        std::ifstream pfile(p + "/idProduct");
        if (vfile && pfile) {
            std::string vs, ps;
            vfile >> vs; pfile >> ps;
            try {
                return {std::stoul(vs, nullptr, 16),
                        std::stoul(ps, nullptr, 16)};
            } catch (...) { return {0u, 0u}; }
        }
        const auto slash = p.find_last_of('/');
        if (slash == 0 || slash == std::string::npos) break;
        p.resize(slash);
    }
    return {0u, 0u};
}

}  // namespace

// ── Free function: enumerate SPARK MAX USB-CDC ports ─────────────────────

std::vector<std::string> list_sparkmax_ports()
{
    constexpr unsigned SPARK_VID = 0x0483;
    constexpr unsigned SPARK_PID = 0xA30E;

    std::vector<std::string> out;
    glob_t g{};
    if (glob("/dev/ttyACM*", 0, nullptr, &g) == 0) {
        for (std::size_t i = 0; i < g.gl_pathc; ++i) {
            const std::string dev = g.gl_pathv[i];
            const std::string base = dev.substr(std::string("/dev/").size());
            const auto vp = read_usb_vid_pid_for_tty(base);
            if (vp.first == SPARK_VID && vp.second == SPARK_PID) {
                out.push_back(dev);
            }
        }
    }
    globfree(&g);
    std::sort(out.begin(), out.end());
    return out;
}

// ── SparkMaxSession ──────────────────────────────────────────────────────

SparkMaxSession::~SparkMaxSession()
{
    close();
}

bool SparkMaxSession::open(const std::string & port)
{
    close();
    position_rot_.store(std::numeric_limits<float>::quiet_NaN());

    const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;

    if (ioctl(fd, TIOCEXCL) != 0) {
        ::close(fd);
        return false;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    if (!configure_115200_raw(fd)) {
        ::close(fd);
        return false;
    }

    fd_   = fd;
    port_ = port;
    rx_buf_.clear();

    running_.store(true);
    tx_thread_ = std::thread(&SparkMaxSession::tx_loop, this);
    rx_thread_ = std::thread(&SparkMaxSession::rx_loop, this);
    return true;
}

void SparkMaxSession::close()
{
    if (!running_.exchange(false)) {
        // Already closed (or never opened); just make sure fd is gone.
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        return;
    }
    if (tx_thread_.joinable()) tx_thread_.join();
    if (rx_thread_.joinable()) rx_thread_.join();
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    port_.clear();
    device_id_.store(-1);
}

void SparkMaxSession::set_target_position(float rotations)
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    target_position_rot_ = rotations;
}

void SparkMaxSession::enable()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    enabled_ = true;
}

void SparkMaxSession::disable()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    enabled_ = false;
}

bool SparkMaxSession::write_all_fd(const std::string & s)
{
    if (fd_ < 0) return false;
    const char * data = s.data();
    std::size_t rem = s.size();
    while (rem > 0) {
        const ssize_t n = ::write(fd_, data, rem);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        data += n;
        rem  -= static_cast<std::size_t>(n);
    }
    return true;
}

void SparkMaxSession::tx_loop()
{
    using namespace sparkmax;
    constexpr uint8_t BROADCAST_BITMASK = 0xFF;  // all 8 device_ids

    while (running_.load()) {
        const int dev = device_id_.load();
        const uint8_t mask =
            (dev >= 0) ? static_cast<uint8_t>(1u << (dev & 0x07))
                       : BROADCAST_BITMASK;

        std::string burst = make_mode_frame(mask);
        burst += ENABLE_FRAME;

        // Snapshot command state under the mutex.
        bool enabled;
        float target;
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            enabled = enabled_;
            target  = target_position_rot_;
        }
        if (enabled && dev >= 0) {
            const uint32_t arb = encode_arbitration_id(
                API_CLASS_SETPOINT,
                API_INDEX_SETPOINT_POSITION,
                static_cast<uint32_t>(dev));
            const auto payload = encode_position_payload(target);
            burst = to_slcan_frame(arb, payload.data(), payload.size()) + burst;
        }

        if (!write_all_fd(burst)) {
            // Drop fd; rx loop will also exit on its next read error.
            // (Caller can re-open the session.)
            running_.store(false);
            break;
        }
        ++tx_count_;
        std::this_thread::sleep_for(TX_PERIOD);
    }
}

void SparkMaxSession::rx_loop()
{
    char buf[512];
    while (running_.load()) {
        const ssize_t n = (fd_ >= 0) ? ::read(fd_, buf, sizeof(buf)) : -1;
        if (n > 0) {
            rx_buf_.append(buf, static_cast<std::size_t>(n));
            std::size_t start = 0;
            for (std::size_t i = 0; i < rx_buf_.size(); ++i) {
                if (rx_buf_[i] == '\r' || rx_buf_[i] == '\n') {
                    if (i > start) {
                        consume_line(rx_buf_.substr(start, i - start));
                    }
                    start = i + 1;
                }
            }
            rx_buf_.erase(0, start);
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK
                            && errno != EINTR) {
            // Hard error; drop out.
            running_.store(false);
            return;
        } else {
            std::this_thread::sleep_for(RX_IDLE_SLEEP);
        }
    }
}

void SparkMaxSession::consume_line(const std::string & line)
{
    using namespace sparkmax;
    auto frame = parse_slcan_extended_rx(line);
    if (!frame) return;

    const uint32_t arb = frame->arbitration_id;
    if (device_id_.load() < 0) {
        device_id_.store(static_cast<int>(arb & 0x3F));
    }

    auto status = identify_periodic_status(arb);
    if (!status) {
        ++other_frame_count_;
        return;
    }

    if (status->status_index == STATUS_INDEX_FAULTS && frame->dlc >= 4) {
        const int16_t raw = decode_status_applied_raw(frame->data.data());
        applied_pct_.store(static_cast<float>(raw) / 32768.0f * 100.0f);
        faults_.store(decode_status_faults(frame->data.data()));
        ++status_0_count_;
    } else if (status->status_index == STATUS_INDEX_POSITION && frame->dlc >= 8) {
        position_rot_.store(decode_status_position_rotations(frame->data.data()));
        ++status_2_count_;
    } else {
        ++other_frame_count_;
    }
}

}  // namespace warrior::hardware

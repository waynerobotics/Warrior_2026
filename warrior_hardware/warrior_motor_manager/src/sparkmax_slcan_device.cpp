#include "warrior_motor_manager/sparkmax_slcan_device.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <thread>

#include "warrior_motor_manager/sparkmax_frame.hpp"

namespace warrior::hardware {

namespace {

bool configure_115200(int fd)
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

}  // namespace

SparkMaxSlcanDevice::~SparkMaxSlcanDevice()
{
    close();
}

bool SparkMaxSlcanDevice::open(const std::string & port, char bitrate_code)
{
    close();

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    if (ioctl(fd_, TIOCEXCL) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    if (!configure_115200(fd_)) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    port_ = port;

    // Confirm we're actually talking to an SLCAN adapter before we attempt to
    // open the CAN channel — the port might be unrelated USB-CDC traffic.
    if (!probe_is_slcan()) {
        close();
        return false;
    }

    // Channel init: close (no-op if not open), set bitrate, open.
    if (!send_raw("C\r"))                                   { close(); return false; }
    if (!send_raw(std::string("S") + bitrate_code + "\r"))  { close(); return false; }
    if (!send_raw("O\r"))                                   { close(); return false; }

    return true;
}

void SparkMaxSlcanDevice::close()
{
    if (fd_ >= 0) {
        send_raw("C\r");   // best-effort channel close
        ::close(fd_);
        fd_ = -1;
    }
    port_.clear();
}

bool SparkMaxSlcanDevice::send_raw(const std::string & cmd)
{
    if (fd_ < 0) return false;
    const char * d = cmd.data();
    std::size_t rem = cmd.size();
    while (rem > 0) {
        const ssize_t n = ::write(fd_, d, rem);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        d   += n;
        rem -= static_cast<std::size_t>(n);
    }
    return true;
}

bool SparkMaxSlcanDevice::probe_is_slcan()
{
    if (fd_ < 0) return false;
    if (!send_raw("V\r")) return false;

    char buf[64];
    std::string acc;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

    while (std::chrono::steady_clock::now() < deadline) {
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n > 0) {
            acc.append(buf, static_cast<std::size_t>(n));
            for (char c : acc) {
                if (c == 'V' || c == 'v') return true;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    return false;
}

bool SparkMaxSlcanDevice::send_position(int can_id, float rotations)
{
    using namespace sparkmax;
    const uint32_t arb_id = encode_arbitration_id(
        API_CLASS_SET_SETPOINT, API_INDEX_SET_POSITION, static_cast<uint32_t>(can_id));
    const auto data = encode_position_payload(rotations);
    return send_raw(to_slcan_frame(arb_id, data.data(), data.size()));
}

std::vector<sparkmax::CanFrame> SparkMaxSlcanDevice::read_pending_frames()
{
    std::vector<sparkmax::CanFrame> out;
    if (fd_ < 0) return out;

    char buf[512];
    while (true) {
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n > 0) {
            rx_buf_.append(buf, static_cast<std::size_t>(n));
        } else {
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                close();
                return out;
            }
            break;
        }
    }

    // SLCAN delimits messages with '\r' (also tolerate '\n').
    std::size_t start = 0;
    for (std::size_t i = 0; i < rx_buf_.size(); ++i) {
        if (rx_buf_[i] == '\r' || rx_buf_[i] == '\n') {
            if (i > start) {
                auto frame = sparkmax::parse_slcan_extended_rx(rx_buf_.substr(start, i - start));
                if (frame) out.push_back(*frame);
            }
            start = i + 1;
        }
    }
    rx_buf_.erase(0, start);
    return out;
}

}  // namespace warrior::hardware

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <thread>

#include "warrior_driver/sparkmax/serial_protocol.hpp"
#include "warrior_driver/arduino/arduino_serial_device.hpp"

namespace warrior::driver {

namespace {

speed_t baud_to_speed(int baud)
{
    switch (baud) {
        case   9600: return B9600;
        case  19200: return B19200;
        case  38400: return B38400;
        case  57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return B115200;
    }
}

bool configure_port(int fd, int baud_rate)
{
    struct termios tio{};
    if (tcgetattr(fd, &tio) != 0) return false;

    cfmakeraw(&tio);

    const speed_t speed = baud_to_speed(baud_rate);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;

    // Non-blocking reads: return immediately whether bytes arrived or not.
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) return false;
    tcflush(fd, TCIOFLUSH);
    return true;
}

}  // namespace

ArduinoSerialDevice::~ArduinoSerialDevice()
{
    close();
}

bool ArduinoSerialDevice::open(const std::string & port, int baud_rate)
{
    close();

    const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::fprintf(stderr, "ArduinoSerialDevice: open(\"%s\") failed: %s\n",
                     port.c_str(), std::strerror(errno));
        return false;
    }

    // Exclusive lock so other processes can't steal bytes (matches pyserial exclusive=True).
    if (ioctl(fd, TIOCEXCL) != 0) {
        ::close(fd);
        return false;
    }

    // Clear O_NONBLOCK after open so VMIN/VTIME drive read behaviour.
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    if (!configure_port(fd, baud_rate)) {
        ::close(fd);
        return false;
    }

    fd_   = fd;
    port_ = port;
    rx_buf_.clear();
    device_name_.clear();
    return true;
}

void ArduinoSerialDevice::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    port_.clear();
    device_name_.clear();
    rx_buf_.clear();
}

bool ArduinoSerialDevice::write_frame(const std::string & frame)
{
    if (fd_ < 0) return false;
    const char * data = frame.data();
    std::size_t remaining = frame.size();
    while (remaining > 0) {
        const ssize_t n = ::write(fd_, data, remaining);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        data      += n;
        remaining -= static_cast<std::size_t>(n);
    }
    return true;
}

std::vector<std::string> ArduinoSerialDevice::read_pending_lines()
{
    std::vector<std::string> out;
    if (fd_ < 0) return out;

    char buf[256];
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

    std::size_t start = 0;
    for (std::size_t i = 0; i < rx_buf_.size(); ++i) {
        if (rx_buf_[i] == '\n' || rx_buf_[i] == '\r') {
            if (i > start) {
                out.emplace_back(rx_buf_.substr(start, i - start));
            }
            start = i + 1;
        }
    }
    rx_buf_.erase(0, start);
    return out;
}

std::optional<std::string> ArduinoSerialDevice::handshake(std::chrono::milliseconds timeout)
{
    if (fd_ < 0) return std::nullopt;
    if (!write_frame(serial_protocol::encode_who())) return std::nullopt;

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        for (const auto & line : read_pending_lines()) {
            const auto fields = serial_protocol::parse_frame(line);
            if (!fields || fields->size() < 2) continue;
            if (fields->at(0) == "NAME") {
                device_name_ = fields->at(1);
                return device_name_;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return std::nullopt;
}

}  // namespace warrior::driver

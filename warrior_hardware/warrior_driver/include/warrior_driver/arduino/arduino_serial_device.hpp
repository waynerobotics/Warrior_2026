#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace warrior::driver {

// Non-blocking ASCII line-based serial port wrapper for Warrior Arduinos.
//
// The constructor does nothing; call open(), then handshake() to discover
// the remote device_name, then write_frame() / read_pending_lines().
//
// Not copyable. Closes the fd on destruction.
class ArduinoSerialDevice
{
public:
    ArduinoSerialDevice() = default;
    ~ArduinoSerialDevice();

    ArduinoSerialDevice(const ArduinoSerialDevice &) = delete;
    ArduinoSerialDevice & operator=(const ArduinoSerialDevice &) = delete;

    // Open the port exclusively at the given baud rate. Returns false on any
    // POSIX error. Does NOT wait for the Arduino bootloader — caller must
    // sleep ~2 s after a successful open before issuing a handshake.
    bool open(const std::string & port, int baud_rate);
    void close();
    bool is_open() const { return fd_ >= 0; }

    const std::string & port() const { return port_; }
    const std::string & device_name() const { return device_name_; }
    void set_device_name(std::string name) { device_name_ = std::move(name); }

    // Write a pre-encoded frame (e.g. from serial_protocol::encode_drive).
    // Returns false on any POSIX write error; caller should close() and rediscover.
    bool write_frame(const std::string & frame);

    // Read everything in the OS RX buffer, split on '\n', and return the
    // complete lines. Partial trailing input is retained for the next call.
    // Never blocks longer than the configured per-fd VTIME.
    std::vector<std::string> read_pending_lines();

    // Send <WHO>, then poll for a <NAME,...> frame until the deadline.
    // Returns the matched name (sets device_name_ too) or nullopt on timeout.
    // Other frames (e.g. streaming <MOT,...>) are discarded during the wait.
    std::optional<std::string> handshake(std::chrono::milliseconds timeout);

private:
    int fd_ = -1;
    std::string port_;
    std::string device_name_;
    std::string rx_buf_;
};

}  // namespace warrior::driver

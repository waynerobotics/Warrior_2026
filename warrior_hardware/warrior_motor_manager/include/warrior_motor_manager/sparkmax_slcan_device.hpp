#pragma once

#include <string>
#include <vector>

#include "warrior_hardware/sparkmax_frame.hpp"

namespace warrior::hardware {

// Wraps a USB-CDC SLCAN adapter (CANable, slcan-firmware STM32, etc.) and
// speaks the SLCAN ASCII protocol. SLCAN bitrate codes:
//   S0=10k S1=20k S2=50k S3=100k S4=125k S5=250k S6=500k S7=800k S8=1M
// FRC / SPARK MAX defaults to 1 Mbps (code '8').
class SparkMaxSlcanDevice
{
public:
    SparkMaxSlcanDevice() = default;
    ~SparkMaxSlcanDevice();

    SparkMaxSlcanDevice(const SparkMaxSlcanDevice &) = delete;
    SparkMaxSlcanDevice & operator=(const SparkMaxSlcanDevice &) = delete;

    // Open the port at 115200, probe with V\r, then send C\r, S<rate>\r, O\r.
    // Returns false on any failure (and leaves the device closed).
    bool open(const std::string & port, char bitrate_code);
    void close();
    bool is_open() const { return fd_ >= 0; }

    const std::string & port() const { return port_; }

    // Probe by sending V\r and waiting for a reply line starting with V or v.
    // Used internally by open(); also exposed for the registry's discovery loop.
    bool probe_is_slcan();

    // Build + transmit a SPARK MAX position-setpoint frame for `can_id`.
    // False on write error; caller should drop the device.
    bool send_position(int can_id, float rotations);

    // Drain the OS RX buffer, split on '\r', and return all parsed extended
    // CAN frames. Non-T lines (e.g. status replies) are silently ignored.
    std::vector<sparkmax::CanFrame> read_pending_frames();

private:
    bool send_raw(const std::string & cmd);

    int fd_ = -1;
    std::string port_;
    std::string rx_buf_;
};

}  // namespace warrior::hardware

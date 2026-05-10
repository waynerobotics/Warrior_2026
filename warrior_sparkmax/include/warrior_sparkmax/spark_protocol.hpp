#pragma once

#include <array>
#include <cstdint>
#include <cstring>

namespace warrior_sparkmax
{

// ---------------------------------------------------------------------------
// SPARK MAX USB/CAN Binary Protocol
// ---------------------------------------------------------------------------
//
// The SPARK MAX USB interface (CDC ACM / /dev/ttyACMx) uses fixed 12-byte
// binary packets in both directions.
//
// Packet layout (all fields little-endian):
//   Bytes [0:3]  = Command ID  (32-bit word encoding the CAN extended ID)
//   Bytes [4:7]  = Data word 0 (payload bytes 0-3)
//   Bytes [8:11] = Data word 1 (payload bytes 4-7)
//
// CAN extended ID bit layout (29 bits packed into 32 bits):
//   Bits [28:24]  Device Type   (5 bits)  — 0x02 = Motor Controller
//   Bits [23:16]  Manufacturer  (8 bits)  — 0x05 = REV Robotics
//   Bits [15:12]  API Class     (4 bits)  — selects command group
//   Bits [11:8]   API Index     (4 bits)  — selects specific command
//   Bits [7:2]    Device ID     (6 bits)  — SPARK MAX CAN ID; DNC for USB
//   Bits [1:0]    Reserved               — 0
//
// On USB, Device ID is ignored (set to 0). All 12 bytes are always sent
// and a 12-byte response is always received.
//
// ⚠️  VERIFICATION NOTE:
//   The API Class/Index values below were derived from the SPARK MAX CAN
//   protocol documentation and community research. Before deploying on real
//   hardware, capture USB traffic with:
//     sudo modprobe usbmon
//     wireshark  (filter: usb.src == "...")
//   while the REV Hardware Client sends the same commands, and compare bytes.
// ---------------------------------------------------------------------------

// Device constants
static constexpr uint32_t SPARK_DEVICE_TYPE   = 0x02u;  // Motor Controller
static constexpr uint32_t SPARK_MANUFACTURER  = 0x05u;  // REV Robotics

// API Classes
static constexpr uint32_t API_CLASS_SETPOINT  = 0x02u;  // setpoint commands
static constexpr uint32_t API_CLASS_HEARTBEAT = 0x06u;  // heartbeat / enable

// API Indices — Setpoint (API Class 0x02)
static constexpr uint32_t API_IDX_DUTY_CYCLE  = 0x00u;  // open-loop duty cycle (-1..1)
static constexpr uint32_t API_IDX_VELOCITY    = 0x01u;  // closed-loop RPM
static constexpr uint32_t API_IDX_POSITION    = 0x02u;  // closed-loop rotations

// API Index — Heartbeat (API Class 0x06)
static constexpr uint32_t API_IDX_HEARTBEAT   = 0x02u;

// USB VID/PID for identifying SPARK MAX ports (STMicro CDC-ACM)
static constexpr uint16_t SPARK_USB_VID = 0x0483u;
static constexpr uint16_t SPARK_USB_PID = 0x5740u;

static constexpr int SPARK_PACKET_BYTES = 12;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build the 32-bit command ID word from API class/index and optional device ID.
inline uint32_t make_cmd_id(uint32_t api_class, uint32_t api_index,
                             uint32_t device_id = 0u)
{
    return (SPARK_DEVICE_TYPE               << 24u) |
           (SPARK_MANUFACTURER              << 16u) |
           ((api_class  & 0xFu)            << 12u) |
           ((api_index  & 0xFu)            <<  8u) |
           ((device_id  & 0x3Fu)           <<  2u);
}

// ---------------------------------------------------------------------------
// Packet
// ---------------------------------------------------------------------------

struct SparkPacket
{
    uint8_t bytes[SPARK_PACKET_BYTES] = {};

    // --- Field writers ---

    void set_cmd_id(uint32_t id)
    {
        bytes[0] = static_cast<uint8_t>(id);
        bytes[1] = static_cast<uint8_t>(id >> 8u);
        bytes[2] = static_cast<uint8_t>(id >> 16u);
        bytes[3] = static_cast<uint8_t>(id >> 24u);
    }

    void set_float32(int offset, float v)
    {
        uint32_t bits;
        std::memcpy(&bits, &v, sizeof(bits));
        bytes[offset + 0] = static_cast<uint8_t>(bits);
        bytes[offset + 1] = static_cast<uint8_t>(bits >>  8u);
        bytes[offset + 2] = static_cast<uint8_t>(bits >> 16u);
        bytes[offset + 3] = static_cast<uint8_t>(bits >> 24u);
    }

    // --- Field readers ---

    float get_float32(int offset) const
    {
        uint32_t bits =
            static_cast<uint32_t>(bytes[offset + 0])        |
            (static_cast<uint32_t>(bytes[offset + 1]) <<  8u) |
            (static_cast<uint32_t>(bytes[offset + 2]) << 16u) |
            (static_cast<uint32_t>(bytes[offset + 3]) << 24u);
        float v;
        std::memcpy(&v, &bits, sizeof(v));
        return v;
    }

    // --- Factory methods ---

    /// Heartbeat packet — must be sent every ~50 ms or the SPARK MAX disables output.
    static SparkPacket heartbeat()
    {
        SparkPacket p;
        p.set_cmd_id(make_cmd_id(API_CLASS_HEARTBEAT, API_IDX_HEARTBEAT));
        // Data bytes: heartbeat counter in bytes [4:7], flags in [8:11].
        // Sending all zeros is accepted; the controller just needs to see the packet.
        return p;
    }

    /// Position setpoint — closed-loop position mode, units: rotations.
    static SparkPacket position_setpoint(float rotations, uint32_t device_id = 0u)
    {
        SparkPacket p;
        p.set_cmd_id(make_cmd_id(API_CLASS_SETPOINT, API_IDX_POSITION, device_id));
        p.set_float32(4, rotations);  // data word 0 = setpoint
        return p;
    }
};

}  // namespace warrior_sparkmax

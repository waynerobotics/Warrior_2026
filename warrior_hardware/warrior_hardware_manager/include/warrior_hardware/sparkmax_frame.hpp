#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>

// REV SPARK MAX CAN protocol — frame builders.
//
// The 29-bit extended-frame arbitration ID is laid out as:
//   [28:24] device type     = 0x02 (motor controller)
//   [23:16] manufacturer    = 0x05 (REV)
//   [15:10] API class       (6 bits)
//   [ 9: 6] API index       (4 bits)
//   [ 5: 0] device CAN ID   (6 bits, 1..62)
//
// Payload layout for "Set Setpoint":
//   bytes 0..3   float32 setpoint    (rotations for position mode)
//   bytes 4..7   reserved / aux FF   (zeroed here)
//
// WARNING: the exact API_INDEX value for position setpoint depends on the
// SPARK MAX firmware version. The constant below matches REV's publicly
// documented 1.5+ protocol but MUST be verified against the firmware on the
// deployed controllers before driving anything physical. Adjust if needed.

namespace warrior::hardware::sparkmax {

constexpr uint32_t DEVICE_TYPE_MOTOR_CONTROLLER = 0x02;
constexpr uint32_t MANUFACTURER_REV             = 0x05;

constexpr uint32_t API_CLASS_SET_SETPOINT = 0x01;
constexpr uint32_t API_INDEX_SET_POSITION = 0x02;  // TODO: verify against REV firmware

inline uint32_t encode_arbitration_id(uint32_t api_class, uint32_t api_index, uint32_t can_id)
{
    return (DEVICE_TYPE_MOTOR_CONTROLLER << 24)
         | (MANUFACTURER_REV             << 16)
         | ((api_class & 0x3F)           << 10)
         | ((api_index & 0x0F)           <<  6)
         | (can_id     & 0x3F);
}

inline std::array<uint8_t, 8> encode_position_payload(float rotations)
{
    std::array<uint8_t, 8> data{};
    std::memcpy(data.data(), &rotations, sizeof(float));
    return data;
}

// SPARK MAX periodic status frames. api_class = 0x06, api_index = 0..7.
//   Status 1: motor velocity (RPM float32 in bytes 0..3)
//   Status 2: motor position (rotations float32 in bytes 0..3)
constexpr uint32_t API_CLASS_PERIODIC_STATUS = 0x06;
constexpr uint32_t STATUS_INDEX_VELOCITY     = 0x01;
constexpr uint32_t STATUS_INDEX_POSITION     = 0x02;

struct PeriodicStatusId
{
    uint32_t can_id;        // 1..62
    uint32_t status_index;  // 0..7
};

// If arb_id is a SPARK MAX periodic status frame, return its (can_id, status_index).
inline std::optional<PeriodicStatusId> identify_periodic_status(uint32_t arb_id)
{
    if ((arb_id >> 24) != DEVICE_TYPE_MOTOR_CONTROLLER) return std::nullopt;
    if (((arb_id >> 16) & 0xFF) != MANUFACTURER_REV)    return std::nullopt;
    if (((arb_id >> 10) & 0x3F) != API_CLASS_PERIODIC_STATUS) return std::nullopt;
    PeriodicStatusId out{};
    out.can_id       = arb_id & 0x3F;
    out.status_index = (arb_id >> 6) & 0x0F;
    if (out.status_index > 7) return std::nullopt;
    return out;
}

// First 4 bytes of a Status 2 frame = motor position in rotations (float32 LE).
inline float decode_status_position_rotations(const uint8_t * data)
{
    float f;
    std::memcpy(&f, data, sizeof(float));
    return f;
}

// First 4 bytes of a Status 1 frame = motor velocity in RPM (float32 LE),
// assuming the SPARK MAX's velocity-conversion-factor is at default (1.0).
inline float decode_status_velocity_rpm(const uint8_t * data)
{
    float f;
    std::memcpy(&f, data, sizeof(float));
    return f;
}

// ─── SLCAN ASCII protocol helpers ────────────────────────────────────────

struct CanFrame
{
    uint32_t arbitration_id = 0;
    uint8_t  dlc            = 0;
    std::array<uint8_t, 8> data{};
};

// Parse one SLCAN RX line. We only handle the extended-frame form ("T...").
// SLCAN may append a 4-hex-digit timestamp after the data bytes if Z1 was sent.
inline std::optional<CanFrame> parse_slcan_extended_rx(const std::string & line)
{
    if (line.size() < 10) return std::nullopt;  // 'T' + 8 id + 1 dlc
    if (line[0] != 'T') return std::nullopt;

    const auto hex_nibble = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    };

    uint32_t id = 0;
    for (int i = 0; i < 8; ++i) {
        const int v = hex_nibble(line[1 + i]);
        if (v < 0) return std::nullopt;
        id = (id << 4) | static_cast<uint32_t>(v);
    }

    const int dlc = hex_nibble(line[9]);
    if (dlc < 0 || dlc > 8) return std::nullopt;
    if (line.size() < static_cast<std::size_t>(10 + dlc * 2)) return std::nullopt;

    CanFrame f{};
    f.arbitration_id = id;
    f.dlc            = static_cast<uint8_t>(dlc);
    for (int i = 0; i < dlc; ++i) {
        const int hi = hex_nibble(line[10 + i * 2]);
        const int lo = hex_nibble(line[10 + i * 2 + 1]);
        if (hi < 0 || lo < 0) return std::nullopt;
        f.data[static_cast<std::size_t>(i)] = static_cast<uint8_t>((hi << 4) | lo);
    }
    return f;
}

// Convert (arbitration_id, payload) into an SLCAN ASCII transmit frame:
//   "T<8-hex-id><1-dec-dlc><N*2-hex-bytes>\r"
inline std::string to_slcan_frame(uint32_t arbitration_id, const uint8_t * data, std::size_t dlc)
{
    char buf[16];
    std::string s;
    s.reserve(1 + 8 + 1 + dlc * 2 + 1);

    std::snprintf(buf, sizeof(buf), "%08X", arbitration_id);
    s.push_back('T');
    s.append(buf, 8);

    std::snprintf(buf, sizeof(buf), "%X", static_cast<unsigned>(dlc));
    s.append(buf, 1);

    for (std::size_t i = 0; i < dlc; ++i) {
        std::snprintf(buf, sizeof(buf), "%02X", data[i]);
        s.append(buf, 2);
    }
    s.push_back('\r');
    return s;
}

}  // namespace warrior::hardware::sparkmax

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
// All constants below come from sniffing REV Hardware Client traffic with
// scripts/sniff_usb.py against the SPARK MAX's USB-CDC SLCAN interface,
// and match what warrior_serial/warrior_serial/nudge_sparks.py uses (which
// is the authoritative working reference — see CLAUDE.md).

namespace warrior::driver::sparkmax {

constexpr uint32_t DEVICE_TYPE_MOTOR_CONTROLLER = 0x02;
constexpr uint32_t MANUFACTURER_REV             = 0x05;

// Set-setpoint frame. REV's frame T020501<dev>... -> api_class=0, api_index=4
// for position setpoint. Payload = float32_le(rotations) + float32_le(arbFF).
constexpr uint32_t API_CLASS_SETPOINT          = 0x00;
constexpr uint32_t API_INDEX_SETPOINT_POSITION = 0x04;

// Compatibility aliases — old SparkMaxSlcanDevice still references these.
// Will be removed once that device is replaced by SparkMaxSession everywhere.
constexpr uint32_t API_CLASS_SET_SETPOINT  = API_CLASS_SETPOINT;
constexpr uint32_t API_INDEX_SET_POSITION  = API_INDEX_SETPOINT_POSITION;

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
    // 4 bytes setpoint + 4 bytes arbFF (zeroed). Matches struct.pack('<ff', x, 0.0).
    std::array<uint8_t, 8> data{};
    std::memcpy(data.data(), &rotations, sizeof(float));
    return data;
}

// SPARK MAX periodic status frames are api_class=0x2E (NOT 0x06 — the public
// REV docs are misleading; sniffed traffic disagrees).
//   Status 0: applied output + faults  (api_index=0)
//   Status 1: motor velocity            (api_index=1)
//   Status 2: motor position            (api_index=2)
constexpr uint32_t API_CLASS_PERIODIC_STATUS = 0x2E;
constexpr uint32_t STATUS_INDEX_FAULTS       = 0x00;
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

// Status 2 payload: motor position float32 LE at byte offset **4** (not 0).
// Bytes 0..3 carry something else (also a float, but not the position).
inline float decode_status_position_rotations(const uint8_t * data)
{
    float f;
    std::memcpy(&f, data + 4, sizeof(float));
    return f;
}

// Status 1 payload: motor velocity RPM float32 LE at byte offset 0
// (assuming velocity-conversion-factor at default 1.0).
inline float decode_status_velocity_rpm(const uint8_t * data)
{
    float f;
    std::memcpy(&f, data, sizeof(float));
    return f;
}

// Status 0 payload bytes 0..1 = int16 LE applied-output (raw, scale by /32768
// for fractional, *100 for percent). Bytes 2..3 = uint16 LE fault bitmask.
inline int16_t  decode_status_applied_raw(const uint8_t * data)
{
    int16_t v;
    std::memcpy(&v, data, sizeof(v));
    return v;
}
inline uint16_t decode_status_faults(const uint8_t * data)
{
    uint16_t v;
    std::memcpy(&v, data + 2, sizeof(v));
    return v;
}

// ─── SLCAN ASCII protocol helpers ────────────────────────────────────────

struct CanFrame
{
    uint32_t arbitration_id = 0;
    uint8_t  dlc            = 0;
    std::array<uint8_t, 8> data{};
};

// Parse one SLCAN RX line. We only handle the extended-frame form ("T...").
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

// ─── Heartbeat helpers ─────────────────────────────────────────────────────
//
// SPARK MAXes go silent (no Status 0/1/2 emission) without seeing periodic
// enable + mode-bitmask "heartbeat" frames. From session-open these must be
// transmitted at ~50 Hz, even before discovering device_id. See CLAUDE.md
// rule §4 for why mode-byte-0 must be (1 << device_id) bitmask.

// Build the SPARK MAX broadcast mode frame for the given device-id bitmask.
//   "T02052C80 8 <bitmask:02X> 00 00 00 00 00 00 00\r"
// Pass `0xFF` for "all 8 possible device_ids" before discovery, narrow to
// (1 << device_id) once known.
inline std::string make_mode_frame(uint8_t bitmask)
{
    char buf[40];
    std::snprintf(buf, sizeof(buf),
                  "T02052C808%02X00000000000000\r",
                  static_cast<unsigned>(bitmask));
    return std::string(buf);
}

// The constant FRC-style enable heartbeat. Same on every SPARK MAX, every
// device_id (it's a broadcast). Send alongside the mode frame.
inline constexpr const char * ENABLE_FRAME = "T000502C0101\r";

}  // namespace warrior::driver::sparkmax

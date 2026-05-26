#pragma once

#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace warrior::hardware::serial_protocol {

// All Warrior wire frames are ASCII: <TYPE,field,field,...>\n

inline std::string encode_who()
{
    return "<WHO>\n";
}

inline std::string encode_drive(const std::string & device_name, int percent)
{
    std::ostringstream oss;
    oss << "<DRV," << device_name << ',' << percent << ">\n";
    return oss.str();
}

// Parse one already-trimmed line. Returns the fields between < and >,
// or nullopt if the line is not a well-formed frame.
inline std::optional<std::vector<std::string>> parse_frame(const std::string & line)
{
    const auto lt = line.find('<');
    const auto gt = line.find('>', lt == std::string::npos ? 0 : lt);
    if (lt == std::string::npos || gt == std::string::npos || gt <= lt + 1) {
        return std::nullopt;
    }

    std::vector<std::string> fields;
    std::string current;
    for (std::size_t i = lt + 1; i < gt; ++i) {
        const char c = line[i];
        if (c == ',') {
            fields.push_back(std::move(current));
            current.clear();
        } else {
            current.push_back(c);
        }
    }
    fields.push_back(std::move(current));
    return fields;
}

}  // namespace warrior::hardware::serial_protocol

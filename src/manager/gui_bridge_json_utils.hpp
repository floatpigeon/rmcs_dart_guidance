#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>

namespace rmcs_dart_guidance::manager {

inline std::string escape_json_string(std::string_view input) {
    std::string output;
    output.reserve(input.size() + 8);

    for (const char c : input) {
        switch (c) {
        case '\\': output += "\\\\"; break;
        case '"': output += "\\\""; break;
        case '\b': output += "\\b"; break;
        case '\f': output += "\\f"; break;
        case '\n': output += "\\n"; break;
        case '\r': output += "\\r"; break;
        case '\t': output += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                output += '?';
            } else {
                output.push_back(c);
            }
            break;
        }
    }

    return output;
}

inline uint64_t current_system_timestamp_ms() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

} // namespace rmcs_dart_guidance::manager

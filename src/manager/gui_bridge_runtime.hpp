#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

namespace rmcs_dart_guidance::manager {

struct GuiBridgeRuntime {
    std::mutex command_mutex;
    std::deque<std::string> pending_commands;

    std::mutex state_mutex;
    std::string latest_snapshot_json;
    uint64_t snapshot_version{0};
    std::deque<std::string> pending_events;
};

} // namespace rmcs_dart_guidance::manager

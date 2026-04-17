#pragma once

#include "manager/core/runtime/action_sequence.hpp"

#include <string>

namespace rmcs_dart_guidance::manager {

class Task : public ActionSequence {
public:
    explicit Task(std::string name, std::string description = "")
        : ActionSequence(std::move(name))
        , description_(std::move(description)) {}

    const std::string& description() const { return description_; }

private:
    std::string description_;
};

} // namespace rmcs_dart_guidance::manager

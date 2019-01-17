#pragma once

#include <lib/command.hpp>

class Subsystem {
public:
    Subsystem() = default;
    virtual void ExecuteCommand(Command& command) {};
};
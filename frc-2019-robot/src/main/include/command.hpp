#pragma once

#include <lib/routine.hpp>

#include <vector>

namespace garage {
    struct Command {
    public:
        double driveForward, driveTurn, flipper, elevatorPosition;
        bool button, hatchIntakeDown;
        std::vector<lib::Routine> routines;
    };
}

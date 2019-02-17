#pragma once

#include <lib/routine.hpp>

#include <vector>
#include <memory>

namespace garage {
    struct Command {
    public:
        double driveForward, driveTurn, flipper, ballIntake, elevatorPosition, test;
        bool button, hatchIntakeDown;
        std::vector<std::shared_ptr<lib::Routine>> routines;
    };
}

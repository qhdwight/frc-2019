#pragma once

#include <lib/routine.hpp>

#include <vector>
#include <memory>

namespace garage {
    struct Command {
    public:
        double driveForward, driveTurn, flipper, ballIntake, test, driveForwardFine, driveTurnFine, elevatorInput;
        bool killSwitch, hatchIntakeDown;
        std::vector<std::shared_ptr<lib::Routine>> routines;
    };
}

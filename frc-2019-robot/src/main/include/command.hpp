#pragma once

#include <lib/routine.hpp>

#include <vector>
#include <memory>

namespace garage {
    struct Command {
    public:
        double driveForward, driveTurn, flipper, ballIntake, test, elevatorInput;
        bool hatchIntakeDown, drivePrecisionEnabled, offTheBooksModeEnabled;
        std::vector<std::shared_ptr<lib::Routine>> routines;
    };
}

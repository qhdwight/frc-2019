#pragma once

#include <lib/routine.hpp>

#include <vector>
#include <memory>

namespace garage {
    struct Command {
    public:
        double driveForward, driveTurn, flipper, ballIntake, elevatorInput, outrigger, outriggerWheel;
        bool hatchIntakeDown, drivePrecisionEnabled, offTheBooksModeEnabled;
        std::vector<std::shared_ptr<lib::Routine>> routines;
    };
}

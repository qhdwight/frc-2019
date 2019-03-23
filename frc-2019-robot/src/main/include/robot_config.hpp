#pragma once

#include <lib/logger.hpp>

namespace garage {
    struct RobotConfig {
        lib::Logger::LogLevel logLevel = lib::Logger::LogLevel::k_Info;
        bool
                shouldOutput = true,
        // Subsystems
                enableElevator = true,
                enableDrive = true,
                enableFlipper = true,
                enableBallIntake = true,
                enableHatchIntake = true,
                enableOutrigger = true;
        double bottomHatchHeight = 4.7,
        /* Rocket */
        // Ball
                rocketBottomBallHeight = 0.0, rocketMiddleBallHeight = 0.0, rocketTopBallHeight = 0.0,
        // Hatch
                rocketMiddleHatchHeight = 53.8, rocketTopHatchHeight = 104.8,
                rocketBottomBallAngle = 0.0, rocketMiddleBallAngle = 0.0, rocketTopBallAngle = 0.0,
        /* Cargo */
                cargoBallHeightDown = 0.0, cargoBallHeightUp = 0.0,
                cargoBallAngle = 0.0,
        /* Intake */
                groundIntakeBallHeight = bottomHatchHeight, loadingIntakeBallHeight = 0.0;
    };
}

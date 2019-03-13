#pragma once

#include <routine/elevator_and_flipper_routine.hpp>

namespace garage {
    class Robot;

    class ResetRoutine : public ElevatorAndFlipperRoutine {
    public:
        ResetRoutine(std::shared_ptr<Robot> robot) : ElevatorAndFlipperRoutine(robot, 0, 90.0, "Reset") {}
    };
}
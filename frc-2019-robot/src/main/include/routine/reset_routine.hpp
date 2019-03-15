#pragma once

#include <routine/elevator_and_flipper_routine.hpp>

namespace garage {
    class ResetRoutine : public ElevatorAndFlipperRoutine {
    public:
        ResetRoutine(std::shared_ptr<Robot> robot) : ElevatorAndFlipperRoutine(robot, ELEVATOR_MIN, FLIPPER_LOWER, "Reset") {}
    };
}
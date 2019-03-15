#pragma once

#include <robot.hpp>
#include <routine/set_flipper_angle_routine.hpp>

namespace garage {
    class StowFlipperRoutine : public SetFlipperAngleRoutine {
    public:
        StowFlipperRoutine(std::shared_ptr<Robot>& robot)
                : SetFlipperAngleRoutine(robot, FLIPPER_STOW_ANGLE, "Stow Flipper") {

        }
    };
}
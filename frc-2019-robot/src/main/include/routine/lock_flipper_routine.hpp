#pragma once

#include <routine/set_flipper_angle_routine.hpp>

#include <lib/sequential_routine.hpp>

namespace garage {
    class LockFlipperRoutine : public lib::SequentialRoutine {
    public:
        LockFlipperRoutine(std::shared_ptr<Robot> robot) : lib::SequentialRoutine(robot, "Lock Flipper", {
                std::make_shared<SetFlipperAngleRoutine>(robot, 180.0),
        }) {

        }

        void Terminate() override;
    };
}
#pragma once

#include <routine/set_flipper_servo_routine.hpp>
#include <routine/set_flipper_angle_routine.hpp>

#include <lib/wait_routine.hpp>
#include <lib/sequential_routine.hpp>

namespace garage {
    class LockFlipperRoutine : public lib::SequentialRoutine {
    public:
        LockFlipperRoutine(std::shared_ptr<Robot> robot) : lib::SequentialRoutine(robot, "Lock Flipper", {
                std::make_shared<SetFlipperAngleRoutine>(robot, 180.0),
                std::make_shared<lib::WaitRoutine>(robot, 200l),
                std::make_shared<SetFlipperServoRoutine>(robot, true),
                std::make_shared<lib::WaitRoutine>(robot, 2000l)
        }) {

        }
    };
}
#pragma once

#include <lib/wait_routine.hpp>

#include <routine/reset_routine.hpp>
#include <routine/set_flipper_servo_routine.hpp>

namespace garage {
    class ResetWithServoRoutine : public ResetRoutine {
    public:
        ResetWithServoRoutine(std::shared_ptr<Robot> robot)
                : ResetRoutine(robot) {
            m_SubRoutines.insert(m_SubRoutines.end(), {
                    std::make_shared<lib::WaitRoutine>(robot, 500l),
                    std::make_shared<SetFlipperServoRoutine>(robot, false),
                    std::make_shared<lib::WaitRoutine>(robot, 2000l)
            });
        }
    };
}
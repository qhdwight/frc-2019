#include <routine/lock_flipper_routine.hpp>

#include <robot.hpp>

namespace garage {
    void LockFlipperRoutine::Terminate() {
        SequentialRoutine::Terminate();
        m_Robot->GetFlipper()->LockServo();
    }
}
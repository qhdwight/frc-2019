#include <routine/set_flipper_servo_routine.hpp>

#include <robot.hpp>

namespace garage {
    void SetFlipperServoRoutine::Start() {
        Routine::Start();
        if (m_Subsystem) {
            if (m_ShouldLock) {
                m_Subsystem->LockServo();
            } else {
                m_Subsystem->UnlockServo();
            }
        }
    }

    SetFlipperServoRoutine::SetFlipperServoRoutine(std::shared_ptr<Robot> robot, bool shouldLock, const std::string& name)
        : SubsystemRoutine(robot, name), m_ShouldLock(shouldLock) {

    }
}
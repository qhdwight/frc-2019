#include <routine/flip_over_routine.hpp>

#include <robot.hpp>

namespace garage {
    FlipOverRoutine::FlipOverRoutine(std::shared_ptr<Robot> robot) : SubsystemRoutine(robot, "Flip Over") {
    }

    void FlipOverRoutine::Start() {
        Routine::Start();
        if (m_Subsystem) {
            const double currentAngle = m_Subsystem->GetAngle();
            m_TargetAngle = currentAngle < FLIPPER_STOW_ANGLE ? FLIPPER_UPPER_ANGLE : FLIPPER_LOWER_ANGLE;
            m_Subsystem->SetAngle(m_TargetAngle);
        }
    }

    void FlipOverRoutine::Terminate() {
        Routine::Terminate();
        if (m_Subsystem) {
            m_Subsystem->Unlock();
        }
    }

    bool FlipOverRoutine::CheckFinished() {
        return m_Subsystem ? m_Subsystem->WithinAngle(m_TargetAngle) : true;
    }
}
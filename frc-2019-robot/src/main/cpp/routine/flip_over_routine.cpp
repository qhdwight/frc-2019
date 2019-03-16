#include <routine/flip_over_routine.hpp>

#include <robot.hpp>

namespace garage {
    FlipOverRoutine::FlipOverRoutine(std::shared_ptr<Robot> robot) : SubsystemRoutine(robot, robot->GetFlipper(), "Flip Over") {
    }

    void FlipOverRoutine::Start() {
        Routine::Start();
        const double currentAngle = m_Subsystem->GetAngle();
        m_TargetAngle = currentAngle < FLIPPER_STOW_ANGLE ? FLIPPER_UPPER_ANGLE : FLIPPER_LOWER_ANGLE;
        if (m_Subsystem) {
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
#include <routine/set_flipper_angle_routine.hpp>

#include <robot.hpp>

namespace garage {
    SetFlipperAngleRoutine::SetFlipperAngleRoutine(std::shared_ptr<Robot>& robot, double angle, const std::string& name)
            : SubsystemRoutine(robot, robot->GetFlipper(), "Stow Flipper"), m_Angle(angle) {

    }

    void SetFlipperAngleRoutine::Begin() {
        Routine::Begin();
        m_Subsystem->SetAngle(m_Angle);
    }

    void SetFlipperAngleRoutine::Terminate() {
        Routine::Terminate();
        m_Subsystem->Unlock();
    }
}
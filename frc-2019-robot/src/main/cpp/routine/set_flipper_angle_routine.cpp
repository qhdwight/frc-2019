#include <routine/set_flipper_angle_routine.hpp>

#include <robot.hpp>

namespace garage {
    SetFlipperAngleRoutine::SetFlipperAngleRoutine(std::shared_ptr<Robot> robot, double angle, const std::string& name)
            : SubsystemRoutine(robot, robot->GetFlipper(), name), m_Angle(angle) {

    }

    void SetFlipperAngleRoutine::Start() {
        Routine::Start();
        m_Subsystem->SetAngle(m_Angle);
    }

    void SetFlipperAngleRoutine::Terminate() {
        Routine::Terminate();
        m_Subsystem->Unlock();
    }

    bool SetFlipperAngleRoutine::CheckFinished() {
        return m_Subsystem->WithinAngle(m_Angle);
    }
}
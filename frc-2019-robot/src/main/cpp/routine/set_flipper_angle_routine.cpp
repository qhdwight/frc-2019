#include <routine/set_flipper_angle_routine.hpp>

#include <robot.hpp>

namespace garage {
    SetFlipperAngleRoutine::SetFlipperAngleRoutine(std::shared_ptr<Robot> robot, double angle, const std::string& name)
            : SubsystemRoutine(robot, name), m_Angle(angle) {
        if (m_Subsystem) {
            m_Subsystem->Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("[%s] Set Flipper Angle: %f", FMT_STR(name), angle));
        }
    }

    void SetFlipperAngleRoutine::Start() {
        Routine::Start();
        if (m_Subsystem) {
            m_Subsystem->SetAngle(m_Angle);
        }
    }

    void SetFlipperAngleRoutine::Terminate() {
        Routine::Terminate();
        if (m_Subsystem) {
            m_Subsystem->Unlock();
        }
    }

    bool SetFlipperAngleRoutine::CheckFinished() {
        return m_Subsystem ? m_Subsystem->WithinAngle(m_Angle) : true;
    }
}
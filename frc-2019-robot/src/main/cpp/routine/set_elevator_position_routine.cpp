#include <routine/set_elevator_position_routine.hpp>

#include <robot.hpp>

namespace garage {
    SetElevatorPositionRoutine::SetElevatorPositionRoutine(std::shared_ptr<Robot> robot, double setPoint, const std::string& name)
            : SubsystemRoutine(robot, robot->GetElevator(), name), m_SetPoint(setPoint) {
        if (m_Subsystem) {
            m_Subsystem->Log(lib::Logger::LogLevel::k_Verbose, lib::Logger::Format("[%s] Set Elevator Set Point: %f", FMT_STR(name), setPoint));
        }
    }

    void SetElevatorPositionRoutine::Start() {
        lib::Routine::Start();
        if (m_Subsystem) {
            m_Subsystem->SetWantedSetPoint(m_SetPoint);
        }
    }

    void SetElevatorPositionRoutine::Terminate() {
        lib::Routine::Terminate();
        if (m_Subsystem) {
            m_Subsystem->Unlock();
        }
    }

    bool SetElevatorPositionRoutine::CheckFinished() {
        return m_Subsystem ? m_Subsystem->WithinPosition(m_SetPoint) : true;
    }
}

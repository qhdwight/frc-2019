#include <routine/set_elevator_position_routine.hpp>

#include <robot.hpp>

namespace garage {
    SetElevatorPositionRoutine::SetElevatorPositionRoutine(std::shared_ptr<Robot> robot, int setPoint, const std::string& name)
            : SubsystemRoutine(robot, robot->GetElevator(), name), m_SetPoint(setPoint) {

    }

    void SetElevatorPositionRoutine::Start() {
        lib::Routine::Start();
        m_Subsystem->SetWantedSetPoint(m_SetPoint);
    }

    void SetElevatorPositionRoutine::Terminate() {
        lib::Routine::Terminate();
        m_Subsystem->Unlock();
    }

    bool SetElevatorPositionRoutine::CheckFinished() {
        return m_Subsystem->WithinPosition(m_SetPoint);
    }
}

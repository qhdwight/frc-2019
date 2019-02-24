#include <routine/set_elevator_position_routine.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

namespace garage {
    SetElevatorPositionRoutine::SetElevatorPositionRoutine(std::shared_ptr<Robot>& robot, const std::string& name, int position)
        : Routine(robot, name), m_Position(position), m_Elevator(robot->GetElevator()) {

    }

    void SetElevatorPositionRoutine::Begin() {
        m_Elevator->Lock();
        m_Elevator->SetElevatorWantedSetPoint(m_Position);
    }

    void SetElevatorPositionRoutine::Terminate() {
        m_Elevator->Unlock();
    }

    bool SetElevatorPositionRoutine::CheckFinished() {
        return m_Elevator->WithinPosition(m_Position);
    }
}

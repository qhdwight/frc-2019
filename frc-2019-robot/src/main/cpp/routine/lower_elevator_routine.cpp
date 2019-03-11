#include <routine/lower_elevator_routine.hpp>

#include <robot.hpp>

namespace garage {
    LowerElevatorRoutine::LowerElevatorRoutine(std::shared_ptr<Robot>& robot)
            : SubsystemRoutine(robot, robot->GetElevator(), "Lower Elevator") {

    }

    void LowerElevatorRoutine::Begin() {
        Routine::Begin();
        m_Subsystem->Lock();
        m_Subsystem->SoftLand();
    }

    void LowerElevatorRoutine::Terminate() {
        Routine::Terminate();
        m_Subsystem->Unlock();
    }
}
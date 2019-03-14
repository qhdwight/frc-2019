#include <routine/lower_elevator_routine.hpp>

#include <robot.hpp>

namespace garage {
    SoftLandElevatorRoutine::SoftLandElevatorRoutine(std::shared_ptr<Robot> robot)
            : SubsystemRoutine(robot, robot->GetElevator(), "Lower Elevator") {

    }

    void SoftLandElevatorRoutine::Start() {
        Routine::Start();
        m_Subsystem->SoftLand();
    }

    void SoftLandElevatorRoutine::Terminate() {
        Routine::Terminate();
        m_Subsystem->Unlock();
    }
}
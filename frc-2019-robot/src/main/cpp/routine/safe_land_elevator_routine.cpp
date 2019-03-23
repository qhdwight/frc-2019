#include <routine/safe_land_elevator_routine.hpp>

#include <robot.hpp>

namespace garage {
    SoftLandElevatorRoutine::SoftLandElevatorRoutine(std::shared_ptr<Robot> robot)
            : SubsystemRoutine(robot, "Lower Elevator") {

    }

    void SoftLandElevatorRoutine::Start() {
        Routine::Start();
        if (m_Subsystem) {
            m_Subsystem->SoftLand();
        }
    }

    void SoftLandElevatorRoutine::Terminate() {
        Routine::Terminate();
        if (m_Subsystem) {
            m_Subsystem->Unlock();
        }
    }
}
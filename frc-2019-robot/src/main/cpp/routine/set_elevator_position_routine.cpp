#include <routine/set_elevator_position_routine.hpp>

namespace garage {
    void SetElevatorPositionRoutine::Begin() {
        lib::Routine::Begin();
        m_Subsystem->Lock();
        m_Robot->GetElevator()->SetWantedSetPoint(m_SetPoint);
    }

    void SetElevatorPositionRoutine::Terminate() {
        lib::Routine::Terminate();
        m_Subsystem->Unlock();
    }

    bool SetElevatorPositionRoutine::CheckFinished() {
        // TODO change back
        return m_Subsystem->WithinPosition(m_SetPoint);
    }
}

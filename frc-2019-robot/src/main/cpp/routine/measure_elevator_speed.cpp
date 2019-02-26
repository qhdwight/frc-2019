#include <routine/measure_elevator_speed.hpp>

namespace garage {
    void MeasureElevatorSpeed::Begin() {
        Routine::Begin();
        m_Subsystem->SetRawOutput(m_Output);
    }

    void MeasureElevatorSpeed::Terminate() {
        Routine::Terminate();
        // TODO safe land
        m_Subsystem->SetRawOutput(0.0);
    }

    bool MeasureElevatorSpeed::CheckFinished() {
        return m_Subsystem->GetElevatorPosition() > 120000;
    }
}
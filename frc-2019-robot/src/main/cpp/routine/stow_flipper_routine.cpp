#include <routine/stow_flipper_routine.hpp>

namespace garage {
    StowRoutine::StowRoutine(std::shared_ptr<Robot>& robot)
            : SubsystemRoutine(robot, robot->GetFlipper(), "Stow Flipper") {

    }

    void StowRoutine::Begin() {
        Routine::Begin();
        m_Subsystem->Stow();
    }

    void StowRoutine::Terminate() {
        Routine::Terminate();
        m_Subsystem->Unlock();
    }
}

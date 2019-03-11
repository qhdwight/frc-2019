#pragma once

#include <lib/subsystem_routine.hpp>

#include <memory>

namespace garage {
    class Elevator;
    class LowerElevatorRoutine : public lib::SubsystemRoutine<Elevator> {
    public:
        LowerElevatorRoutine(std::shared_ptr<Robot>& robot);

    private:
        void Begin() override;

        void Terminate() override;
    };
}
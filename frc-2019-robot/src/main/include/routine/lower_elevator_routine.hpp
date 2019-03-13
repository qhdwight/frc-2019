#pragma once

#include <lib/subsystem_routine.hpp>

#include <memory>

namespace garage {
    class Robot;
    class Elevator;
    class SoftLandElevatorRoutine : public lib::SubsystemRoutine<Elevator> {
    public:
        SoftLandElevatorRoutine(std::shared_ptr<Robot>& robot);

    private:
        void Start() override;

        void Terminate() override;
    };
}
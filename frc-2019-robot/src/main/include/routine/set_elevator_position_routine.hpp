#pragma once

#include <lib/subsystem_routine.hpp>

#include <memory>

namespace garage {
    class Elevator;

    class SetElevatorPositionRoutine : public lib::SubsystemRoutine<Elevator> {
    protected:
        double m_SetPoint = 0;

        bool CheckFinished() override;

    public:
        SetElevatorPositionRoutine(std::shared_ptr<Robot> robot, double setPoint, const std::string& name = "Set Elevator Position");

        void Start() override;

        void Terminate() override;
    };
}
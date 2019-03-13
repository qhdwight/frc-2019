#pragma once

#include <lib/subsystem_routine.hpp>

#include <memory>

namespace garage {
    class Robot;

    class Elevator;

    class SetElevatorPositionRoutine : public lib::SubsystemRoutine<Elevator> {
    protected:
        int m_SetPoint = 0;

        bool CheckFinished() override;

    public:
        SetElevatorPositionRoutine(std::shared_ptr<Robot>& robot, int setPoint, const std::string& name = "Set Elevator Position");

        void Start() override;

        void Terminate() override;
    };
}
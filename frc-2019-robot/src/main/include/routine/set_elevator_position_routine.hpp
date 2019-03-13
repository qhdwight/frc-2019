#pragma once

#include <lib/subsystem_routine.hpp>

#include <memory>

namespace garage {
    class Robot;
    class Elevator;
    class SetElevatorPositionRoutine : public lib::SubsystemRoutine<Elevator> {
    private:
        int m_SetPoint;

    public:
        SetElevatorPositionRoutine(std::shared_ptr<Robot>& robot, int setPoint, const std::string& name);

        void Start() override;

        void Terminate() override;

        bool CheckFinished() override;
    };
}
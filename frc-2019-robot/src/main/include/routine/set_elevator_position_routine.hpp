#pragma once

#include <robot.hpp>

#include <lib/subsystem_routine.hpp>

#include <memory>

namespace garage {
    class SetElevatorPositionRoutine : public lib::SubsystemRoutine<Elevator> {
    private:
        int m_SetPoint;

    public:
        SetElevatorPositionRoutine(std::shared_ptr<Robot>& robot, const std::string& name, int setPoint)
                : SubsystemRoutine(robot, robot->GetElevator(), name), m_SetPoint(setPoint) {}

        void Begin() override;

        void Terminate() override;

        bool CheckFinished() override;
    };
}
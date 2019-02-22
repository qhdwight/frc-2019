#pragma once

#include <lib/routine.hpp>

#include <subsystem/elevator.hpp>

#include <memory>

namespace garage {
    class SetElevatorPositionRoutine : public lib::Routine {
    private:
        std::shared_ptr<Elevator> m_Elevator;
        int m_Position;
    public:
        SetElevatorPositionRoutine(std::shared_ptr<Robot>& robot, const std::string& name, int position);

        void Begin() override;

        void Terminate() override;

        bool CheckFinished() override;
    };
}
#pragma once

#include <robot.hpp>
#include <subsystem/elevator.hpp>

#include <lib/subsystem_routine.hpp>

#include <memory>

namespace garage {
    class MeasureElevatorSpeed : public lib::SubsystemRoutine<Elevator> {
    private:
        double m_Output;

    public:
        MeasureElevatorSpeed(std::shared_ptr<Robot>& robot, const std::string& name, double output)
                : SubsystemRoutine(robot, robot->GetElevator(), name), m_Output(output) {}

        void Begin() override;

        void Terminate() override;

        bool CheckFinished() override;
    };
}
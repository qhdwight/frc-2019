#pragma once

#include <robot.hpp>

#include <lib/subsystem_routine.hpp>

#include <memory>
#include <vector>

namespace garage {
    namespace test {
        class MeasureElevatorSpeed : public lib::SubsystemRoutine<Elevator> {
        protected:
            std::vector<double> m_Velocities;
            double m_Output;

        public:
            MeasureElevatorSpeed(std::shared_ptr<Robot>& robot, const std::string& name, double output)
                    : SubsystemRoutine(robot, name), m_Output(output) {}

            void Start() override;

            void Update() override;

            void Terminate() override;

            bool CheckFinished() override;
        };
    }
}
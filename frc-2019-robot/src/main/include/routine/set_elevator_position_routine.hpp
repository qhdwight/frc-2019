#pragma once

#include <lib/routine.hpp>

namespace garage {
    namespace test {
        class SetElevatorPositionRoutine : public lib::Routine {
        private:
            int m_Position;
        public:
            SetElevatorPositionRoutine(std::shared_ptr<Robot>& robot, const std::string& name, int position);

            void Begin() override;

            void Terminate() override;

            bool CheckFinished() override;
        };
    }
}
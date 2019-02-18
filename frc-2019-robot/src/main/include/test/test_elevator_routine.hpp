#pragma once

#include <lib/routine.hpp>

namespace garage {
    namespace test {
        class TestElevatorRoutine : public lib::Routine {
        private:
            int m_Position;
        public:
            TestElevatorRoutine(std::shared_ptr<Robot>& robot, std::string name, int position);

            void Begin() override;

            void Terminate() override;

            bool CheckFinished() override;
        };
    }
}
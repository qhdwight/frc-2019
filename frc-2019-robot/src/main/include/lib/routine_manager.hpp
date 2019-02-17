#pragma once

#include <command.hpp>

#include <lib/routine.hpp>

#include <test/test_elevator_routine.hpp>

#include <queue>
#include <memory>
#include <utility>

namespace garage {
    class Robot;
    namespace lib {
        class RoutineManager {
        private:
            std::shared_ptr<Robot> m_Robot;
            std::queue<std::shared_ptr<Routine>> m_QueuedRoutines;
            std::shared_ptr<Routine> m_TestElevatorRoutine;
            std::shared_ptr<Routine> m_ActiveRoutine;
        public:
            RoutineManager(std::shared_ptr<Robot>& robot);

            void AddRoutinesFromCommand(const Command& command);

            void Update();

            std::shared_ptr<Routine> GetTestElevatorRoutine() {
                if (!m_TestElevatorRoutine)
                    m_TestElevatorRoutine = std::make_shared<test::TestElevatorRoutine>(m_Robot, 6000);
                return m_TestElevatorRoutine;
            }
        };
    }
}
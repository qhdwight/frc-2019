#pragma once

#include <command.hpp>

#include <lib/routine.hpp>

#include <test/test_elevator_routine.hpp>

#include <queue>
#include <memory>
#include <utility>

namespace garage {
    namespace lib {
        class RoutineManager {
        private:
            std::queue<std::shared_ptr<Routine>> m_QueuedRoutines;
//            std::shared_ptr<Routine> m_TestElevatorRoutsine = std::make_shared<test::TestElevatorRoutine>();
            std::shared_ptr<Routine> m_ActiveRoutine;
        public:
            void AddRoutinesFromCommand(const Command& command);

            void Update();

//            std::shared_ptr<Routine> GetTestElevatorRoutine() {
//                return m_TestElevatorRoutine;
//            }
        };
    }
}
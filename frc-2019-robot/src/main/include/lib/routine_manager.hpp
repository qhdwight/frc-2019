#pragma once

#include <command.hpp>

#include <lib/routine.hpp>

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
            std::shared_ptr<Routine> m_ActiveRoutine;
        public:
            RoutineManager(std::shared_ptr<Robot>& robot);

            void AddRoutinesFromCommand(const Command& command);

            void Update();

            void TerminateAllRoutines();
        };
    }
}
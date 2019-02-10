#pragma once

#include <command.hpp>

#include <lib/routine.hpp>

#include <queue>
#include <utility>

namespace garage {
    namespace lib {
        class RoutineManager {
        private:
            std::queue<Routine> m_QueuedRoutines;
            std::pair<bool, Routine> m_ActiveRoutine;
        public:
            void AddRoutinesFromCommand(const Command& command);

            void Update();
        };
    }
}
#pragma once

#include <lib/routine.hpp>

#include <queue>
#include <command.hpp>

namespace garage {
    namespace lib {
        class RoutineManager {
        private:
            std::queue<Routine> m_Routines;
        public:
            void AddRoutinesFromCommand(const Command& command);
        };
    }
}
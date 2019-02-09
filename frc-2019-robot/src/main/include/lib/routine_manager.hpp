#pragma once

#include <lib/routine.hpp>

#include <queue>

namespace garage {
    namespace lib {
        class RoutineManager {
        private:
            std::queue<Routine> m_Routines;
        public:
            void AddRoutine(const Routine& routine) {
                m_Routines.push(routine);
            }
        };
    }
}
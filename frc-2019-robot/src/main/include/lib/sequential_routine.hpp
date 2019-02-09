#pragma once

#include <lib/routine.hpp>

#include <array>

#define MAX_SEQUENTIAL_ROUTINES 10

namespace garage {
    namespace lib {
        class SequentialRoutine : public Routine {
        private:
            std::array<Routine, MAX_SEQUENTIAL_ROUTINES> m_SubRoutines;
        public:
            bool IsFinished() override;
        };
    }
}
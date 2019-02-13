#pragma once

#include <lib/routine.hpp>

#include <array>

#define MAX_SEQUENTIAL_ROUTINES 10

namespace garage {
    namespace lib {
        class SequentialRoutine : public Routine {
        private:
            std::array<std::pair<bool, Routine>, MAX_SEQUENTIAL_ROUTINES> m_SubRoutines;
        public:
//            SequentialRoutine(std::initializer_list<Routine> )

            bool IsFinished() override;
        };
    }
}
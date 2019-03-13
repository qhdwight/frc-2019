#pragma once

#include <lib/multi_routine.hpp>

namespace garage {
    namespace lib {
        class SequentialRoutine : public MultiRoutine {
        protected:
            unsigned int m_CurrentRoutineIndex = 0;
        public:
            using MultiRoutine::MultiRoutine;

            void Begin() override;

            void Update() override;

            bool CheckFinished() override;

            void Terminate() override;
        };
    }
}
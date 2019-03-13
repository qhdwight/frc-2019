#pragma once

#include <lib/multi_routine.hpp>

namespace garage {
    namespace lib {
        class SequentialRoutine : public MultiRoutine {
        protected:
            unsigned int m_CurrentRoutineIndex = 0;

            void Update() override;

            bool CheckFinished() override;

        public:
            using MultiRoutine::MultiRoutine;

            void Start() override;

            void Terminate() override;
        };
    }
}
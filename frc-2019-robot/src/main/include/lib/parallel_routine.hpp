#pragma once

#include <lib/multi_routine.hpp>

namespace garage {
    namespace lib {
        class ParallelRoutine : public MultiRoutine {
        public:
            using MultiRoutine::MultiRoutine;

            void Start() override;

            void Update() override;

            bool CheckFinished() override;

            void Terminate() override;
        };
    }
}
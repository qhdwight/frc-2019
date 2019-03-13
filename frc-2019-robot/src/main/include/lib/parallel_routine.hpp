#pragma once

#include <lib/multi_routine.hpp>

namespace garage {
    namespace lib {
        class ParallelRoutine : public MultiRoutine {
        protected:
            void Update() override;

            bool CheckFinished() override;

        public:
            using MultiRoutine::MultiRoutine;

            void Start() override;

            void Terminate() override;
        };
    }
}
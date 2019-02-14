#pragma once

#include <lib/routine.hpp>

#include <vector>
#include <memory>

namespace garage {
    namespace lib {
        class MultiRoutine : public Routine {
        protected:
            std::vector<std::shared_ptr<Routine>> m_SubRoutines;
        public:
            void Begin() override;

            void Update() override;

            bool CheckFinished() override;
        };
    }
}
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
            MultiRoutine(std::shared_ptr<Robot>, std::string name, std::vector<std::shared_ptr<Routine>>&& routines);

            void Begin() override;

            void Update() override;

            bool CheckFinished() override;
        };
    }
}
#include <lib/multi_routine.hpp>

#include <algorithm>

namespace garage {
    namespace lib {
        void MultiRoutine::Begin() {
            for (auto& routine : m_SubRoutines)
                routine->Begin();
        }

        void MultiRoutine::Update() {
            for (auto& routine : m_SubRoutines)
                routine->Update();
        }

        bool MultiRoutine::CheckFinished() {
            return std::all_of(m_SubRoutines.begin(), m_SubRoutines.end(),
                               [](auto& routine) {
                                   return routine->CheckFinished();
                               });
        }
    }
}
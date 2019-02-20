#include <lib/parallel_routine.hpp>

#include <algorithm>

namespace garage {
    namespace lib {
        void ParallelRoutine::Begin() {
            Routine::Begin();
            for (auto& routine : m_SubRoutines)
                routine->Begin();
        }

        void ParallelRoutine::Update() {
            Routine::Update();
            for (auto& routine : m_SubRoutines)
                routine->Update();
        }

        bool ParallelRoutine::CheckFinished() {
            return std::all_of(m_SubRoutines.begin(), m_SubRoutines.end(),
                               [](auto& routine) {
                                   return routine->CheckFinished();
                               });
        }

        void ParallelRoutine::Terminate() {
            for (auto& routine : m_SubRoutines)
                routine->Terminate();
        }
    }
}

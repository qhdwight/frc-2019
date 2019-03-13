#include <lib/parallel_routine.hpp>

#include <algorithm>

namespace garage {
    namespace lib {
        void ParallelRoutine::Start() {
            Routine::Start();
            for (auto& routine : m_SubRoutines) {
                routine->Start();
            }
        }

        void ParallelRoutine::Update() {
            Routine::Update();
            for (auto& routine : m_SubRoutines) {
                routine->Periodic();
            }
        }

        bool ParallelRoutine::CheckFinished() {
            return std::all_of(m_SubRoutines.begin(), m_SubRoutines.end(),
                               [](auto& routine) {
                                   return routine->IsFinished();
                               });
        }

        void ParallelRoutine::Terminate() {
            for (auto& routine : m_SubRoutines)
                routine->Terminate();
        }
    }
}

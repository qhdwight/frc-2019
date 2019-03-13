#include <lib/sequential_routine.hpp>

namespace garage {
    namespace lib {
        void SequentialRoutine::Begin() {
            Routine::Begin();
            if (!m_SubRoutines.empty()) {
                m_SubRoutines.front()->Begin();
            }
        }

        void SequentialRoutine::Update() {
            if (m_CurrentRoutineIndex < m_SubRoutines.size()) {
                auto& currentRoutine = m_SubRoutines[m_CurrentRoutineIndex];
                currentRoutine->Update();
                if (currentRoutine->CheckFinished()) {
                    currentRoutine->Terminate();
                    m_CurrentRoutineIndex++;
                    if (m_CurrentRoutineIndex < m_SubRoutines.size()) {
                        auto& nextRoutine = m_SubRoutines[m_CurrentRoutineIndex];
                        nextRoutine->Begin();
                    } else {
                        Terminate();
                    }
                }
            }
        }

        bool SequentialRoutine::CheckFinished() {
            return m_CurrentRoutineIndex >= m_SubRoutines.size();
        }

        void SequentialRoutine::Terminate() {
            if (m_CurrentRoutineIndex < m_SubRoutines.size()) {
                auto& currentRoutine = m_SubRoutines[m_CurrentRoutineIndex];
                currentRoutine->Terminate();
            }
        }
    }
}

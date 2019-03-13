#include <lib/sequential_routine.hpp>

namespace garage {
    namespace lib {
        void SequentialRoutine::Start() {
            Routine::Start();
            if (!m_SubRoutines.empty()) {
                m_SubRoutines.front()->Start();
            }
        }

        void SequentialRoutine::Update() {
            if (m_CurrentRoutineIndex < m_SubRoutines.size()) {
                auto& currentRoutine = m_SubRoutines[m_CurrentRoutineIndex];
                if (currentRoutine->Periodic()) {
                    m_CurrentRoutineIndex++;
                    if (m_CurrentRoutineIndex < m_SubRoutines.size()) {
                        auto& nextRoutine = m_SubRoutines[m_CurrentRoutineIndex];
                        nextRoutine->Start();
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

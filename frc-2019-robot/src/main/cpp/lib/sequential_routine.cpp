#include <lib/sequential_routine.hpp>

namespace garage {
    namespace lib {
        void SequentialRoutine::Update() {
            if (m_CurrentRoutineIndex < m_SubRoutines.size()) {
                auto& routine = m_SubRoutines[m_CurrentRoutineIndex];
                routine->Update();
                if (routine->CheckFinished())
                    m_CurrentRoutineIndex++;
            }
        }

        bool SequentialRoutine::CheckFinished() {
            return m_CurrentRoutineIndex >= m_SubRoutines.size();
        }
    }
}

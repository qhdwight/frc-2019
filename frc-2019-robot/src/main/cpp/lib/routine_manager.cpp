#include <lib/routine_manager.hpp>

namespace garage {
    namespace lib {
        void RoutineManager::AddRoutinesFromCommand(const Command& command) {
            for (const auto& routine : command.routines)
                m_QueuedRoutines.push(routine);
        }

        void RoutineManager::Update() {
            if (m_ActiveRoutine) {
                m_ActiveRoutine->Update();
                if (m_ActiveRoutine->CheckFinished())
                    m_ActiveRoutine = nullptr;
            }
            if (!m_QueuedRoutines.empty() && !m_ActiveRoutine) {
                m_ActiveRoutine = m_QueuedRoutines.front();
                m_QueuedRoutines.pop();
            }
        }
    }
}

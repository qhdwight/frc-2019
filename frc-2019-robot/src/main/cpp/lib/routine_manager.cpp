#include <lib/routine_manager.hpp>

namespace garage {
    namespace lib {
        void RoutineManager::AddRoutinesFromCommand(const Command& command) {
            for (const auto& routine : command.routines)
                m_QueuedRoutines.push(routine);
        }

        void RoutineManager::Update() {
            if (m_ActiveRoutine.first) {
                auto& routine = m_ActiveRoutine.second;
                routine.Update();
                if (routine.IsFinished()) {
                    m_ActiveRoutine.first = false;
                }
            }
            if (!m_ActiveRoutine.first && !m_QueuedRoutines.empty()) {
                m_ActiveRoutine.second = m_QueuedRoutines.front();
                m_QueuedRoutines.pop();
                m_ActiveRoutine.first = true;
            }
        }
    }
}

#include <lib/routine_manager.hpp>

namespace garage {
    namespace lib {
        RoutineManager::RoutineManager(std::shared_ptr<Robot>& robot) : m_Robot(robot) {

        }

        void RoutineManager::AddRoutinesFromCommand(const Command& command) {
            for (auto& routine : command.routines)
                m_QueuedRoutines.push(routine);
        }

        void RoutineManager::Update() {
            if (m_ActiveRoutine) {
                m_ActiveRoutine->Update();
                if (m_ActiveRoutine->CheckFinished()) {
                    m_ActiveRoutine->Terminate();
                    m_ActiveRoutine.reset();
                }
            }
            if (!m_QueuedRoutines.empty() && !m_ActiveRoutine) {
                m_ActiveRoutine = m_QueuedRoutines.front();
                m_ActiveRoutine->Begin();
                m_QueuedRoutines.pop();
            }
        }
    }
}

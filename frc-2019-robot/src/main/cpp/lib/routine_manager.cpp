#include <lib/routine_manager.hpp>

#include <robot.hpp>

namespace garage {
    namespace lib {
        RoutineManager::RoutineManager(std::shared_ptr<Robot>& robot) : m_Robot(robot) {

        }

        void RoutineManager::AddRoutinesFromCommand(Command& command) {
            for (auto& routine : command.routines)
                AddRoutine(routine);
        }

        void RoutineManager::AddRoutine(std::shared_ptr<Routine> routine) {
            // Make sure we are not adding the same exact routine twice
            if (std::find(m_QueuedRoutines.begin(), m_QueuedRoutines.end(), routine) == m_QueuedRoutines.end()) {
                m_QueuedRoutines.push_back(routine);
            }
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
                m_QueuedRoutines.pop_front();
            }
        }

        void RoutineManager::TerminateAllRoutines() {
            TerminateActiveRoutine();
            while (!m_QueuedRoutines.empty()) {
                auto routine = m_QueuedRoutines.front();
                routine->Terminate();
                m_QueuedRoutines.pop_front();
            }
        }

        void RoutineManager::Reset() {
            m_ActiveRoutine.reset();
            m_QueuedRoutines.clear();
        }

        void RoutineManager::TerminateActiveRoutine() {
            if (m_ActiveRoutine) {
                m_ActiveRoutine->Terminate();
                m_ActiveRoutine.reset();
            }
        }
    }
}

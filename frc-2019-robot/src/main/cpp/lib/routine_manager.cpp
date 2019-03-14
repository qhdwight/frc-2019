#include <lib/routine_manager.hpp>

#include <robot.hpp>

namespace garage {
    namespace lib {
        RoutineManager::RoutineManager(std::shared_ptr<Robot>& robot) : m_Robot(robot) {

        }

        void RoutineManager::AddRoutinesFromCommand(Command& command) {
            for (auto& routine : command.routines) {
                AddRoutine(routine);
            }
        }

        void RoutineManager::AddRoutine(std::shared_ptr<Routine> routine) {
            // Make sure we are not adding the same exact routine twice
            if (!routine) {
                Logger::Log(Logger::LogLevel::k_Error, "Trying to add a null routine");
                return;
            }
            // TODO std function? who writes their own algorithms anyways
            for (auto& existingRoutine : m_QueuedRoutines) {
                if (routine == existingRoutine) {
                    return;
                }
            }
            m_QueuedRoutines.push_back(routine);
        }

        void RoutineManager::Update() {
            if (m_ActiveRoutine) {
                if (m_ActiveRoutine->Periodic()) {
                    m_ActiveRoutine.reset();
                }
            }
            if (!m_QueuedRoutines.empty() && !m_ActiveRoutine) {
                m_ActiveRoutine = m_QueuedRoutines.front();
                m_ActiveRoutine->Start();
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

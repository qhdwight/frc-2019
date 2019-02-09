#include <lib/routine_manager.hpp>

namespace garage {
    namespace lib {
        void RoutineManager::AddRoutinesFromCommand(const Command& command) {
            for (const auto& routine : command.routines) {
                m_Routines.push(routine);
            }
        }
    }
}

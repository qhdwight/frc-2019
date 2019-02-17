#include <lib/subsystem.hpp>

namespace garage {
    namespace lib {
        Subsystem::Subsystem(std::shared_ptr<Robot>& robot) : m_Robot(robot) {
        }

        void Subsystem::ExecuteCommand(Command& command) {
            m_LastCommand = command;
        }

        void Subsystem::Lock() {
            m_IsLocked = true;
        }

        void Subsystem::Unlock() {
            m_IsLocked = false;
        }
    }
}

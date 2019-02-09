#include <lib/subsystem.hpp>

namespace garage {
    namespace lib {
        void Subsystem::ExecuteCommand(Command& command) {
            m_LastCommand = command;
        }
    }
}

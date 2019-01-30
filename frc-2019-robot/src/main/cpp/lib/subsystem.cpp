#include <lib/subsystem.hpp>

namespace garage {
    namespace lib {
        Subsystem::Subsystem(std::shared_ptr<Robot>& robot) : m_Robot(robot) {
            Initialize();
        }

        void Subsystem::ExecuteCommand(Command& command) {
            m_LastCommand = command;
        }
    }
}

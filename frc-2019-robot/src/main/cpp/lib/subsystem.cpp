#include <lib/subsystem.hpp>

namespace garage {
    namespace lib {
        Subsystem::Subsystem(std::shared_ptr<Robot>& robot) : m_Robot(robot), enable_shared_from_this() {
            shared_from_this()->Initialize();
        }
    }
}

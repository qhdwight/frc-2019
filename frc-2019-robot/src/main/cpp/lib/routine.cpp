#include <lib/routine.hpp>

#include <robot.hpp>

#include <lib/logger.hpp>

namespace garage {
    namespace lib {
        Routine::Routine(std::shared_ptr<Robot>& robot, const std::string& name) : m_Robot(robot), m_Name(name) {

        }

        void Routine::Begin() {
            m_Robot->GetLogger()->Log(LogLevel::k_Info, "Starting routine");
        }

        void Routine::Terminate() {
            m_Robot->GetLogger()->Log(LogLevel::k_Info, "Terminated routine");
        }
    }
}

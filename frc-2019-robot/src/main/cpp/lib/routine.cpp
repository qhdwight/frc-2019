#include <robot.hpp>

#include <lib/logger.hpp>
#include <lib/routine.hpp>

namespace garage {
    namespace lib {
        Routine::Routine(std::shared_ptr<Robot>& robot, std::string name) : m_Robot(robot), m_Name(std::move(name)) {

        }

        void Routine::Begin() {
            m_Robot->GetLogger()->Log(LogLevel::kInfo, "Starting routine");
        }

        void Routine::Terminate() {
            m_Robot->GetLogger()->Log(LogLevel::kInfo, "Terminated routine");
        }
    }
}

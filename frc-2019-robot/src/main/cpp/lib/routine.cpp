#include <lib/routine.hpp>

#include <robot.hpp>

#include <lib/logger.hpp>

namespace garage {
    namespace lib {
        Routine::Routine(std::shared_ptr<Robot>& robot, const std::string& name) : m_Robot(robot), m_Name(name) {

        }

        void Routine::Begin() {
            Logger::Log(Logger::LogLevel::k_Info, Logger::Format("[%s] Starting routine", m_Name.c_str()));
        }

        void Routine::Terminate() {
            Logger::Log(Logger::LogLevel::k_Info, Logger::Format("[%s] Terminating routine", m_Name.c_str()));
        }
    }
}

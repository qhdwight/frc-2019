#include <lib/routine.hpp>

#include <robot.hpp>

#include <lib/logger.hpp>

namespace garage {
    namespace lib {
        Routine::Routine(std::shared_ptr<Robot>& robot, const std::string& name) : m_Robot(robot), m_Name(name) {

        }

        void Routine::Start() {
            Logger::Log(Logger::LogLevel::k_Info, Logger::Format("[%s] Starting routine", FMT_STR(m_Name)));
            m_Running = true;
        }

        void Routine::Terminate() {
            Logger::Log(Logger::LogLevel::k_Info, Logger::Format("[%s] Terminating routine", FMT_STR(m_Name)));
            m_Running = false;
        }

        void Routine::Periodic() {
            if (m_Running) {
                Update();
            }
        }
    }
}

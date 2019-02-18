#include <utility>

#include <robot.hpp>

#include <lib/subsystem.hpp>

#include <limits>

namespace garage {
    namespace lib {
        Subsystem::Subsystem(std::shared_ptr<Robot>& robot, std::string subsystemName) : m_Robot(robot), m_SubsystemName(std::move(subsystemName)) {
            Log(lib::LogLevel::kInfo, "Subsystem Initialized");
        }

        void Subsystem::ExecuteCommand(Command& command) {
        }

        void Subsystem::Lock() {
            m_IsLocked = true;
        }

        void Subsystem::Unlock() {
            m_IsLocked = false;
        }

        void Subsystem::Update(Command& command) {
            AdvanceSequence();
            if (m_SequenceNumber % SPACED_UPDATE_INTERVAL == 0)
                SpacedUpdate();
            ExecuteCommand(command);
            SetLastCommand(command);
        }

        void Subsystem::AdvanceSequence() {
            if (m_SequenceNumber < std::numeric_limits<unsigned long>::max())
                m_SequenceNumber++;
            else
                m_SequenceNumber = 0;
        }

        void Subsystem::SetLastCommand(Command& command) {
            m_LastCommand = command;
        }

        void Subsystem::SpacedUpdate() {
            Log(lib::LogLevel::kInfo, std::to_string(m_LastCommand.driveForward) + ", " + std::to_string(m_LastCommand.driveTurn));
        }

        void Subsystem::Log(lib::LogLevel logLevel, std::string log) {
            m_Robot->GetLogger()->Log(logLevel, "[" + m_SubsystemName + "]: " + log);
        }
    }
}

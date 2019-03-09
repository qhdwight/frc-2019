#include <lib/subsystem.hpp>

#include <robot.hpp>

#include <utility>
#include <limits>

namespace garage {
    namespace lib {
        Subsystem::Subsystem(std::shared_ptr<Robot>& robot, const std::string& subsystemName)
                : m_Robot(robot), m_NetworkTable(robot->GetNetworkTable()->GetSubTable(subsystemName)), m_SubsystemName(subsystemName) {
            Log(Logger::LogLevel::k_Info, "Subsystem Initialized");
        }

        void Subsystem::Lock() {
            Log(lib::Logger::LogLevel::k_Info, "Locked");
            m_IsLocked = true;
            OnLock();
        }

        void Subsystem::Unlock() {
            Log(lib::Logger::LogLevel::k_Info, "Unlocked");
            m_IsLocked = false;
            OnUnlock();
        }

        void Subsystem::Periodic() {
            auto command = m_Robot->GetLatestCommand();
            if (m_IsLocked && ShouldUnlock(command))
                Unlock();
            AdvanceSequence();
            if (m_SequenceNumber % SPACED_UPDATE_INTERVAL == 0)
                SpacedUpdate(command);
            Update();
            if (m_IsLocked)
                UpdateLocked();
            else
                UpdateUnlocked(command);
            m_LastCommand = command;
        }

        void Subsystem::AdvanceSequence() {
            if (m_SequenceNumber < std::numeric_limits<unsigned long>::max())
                m_SequenceNumber++;
            else
                m_SequenceNumber = 0;
        }

        void Subsystem::Log(Logger::LogLevel logLevel, const std::string& log) {
            Logger::Log(logLevel, Logger::Format("[%s] %s", m_SubsystemName.c_str(), log.c_str()));
        }

        void Subsystem::LogSample(Logger::LogLevel logLevel, const std::string& log, int frequency) {
            if (m_SequenceNumber % frequency == 0)
                Log(logLevel, log);
        }

        bool Subsystem::ShouldUnlock(Command& command) {
            return true;
        }

        void Subsystem::AddNetworkTableListener(const std::string& entryName, const double defaultValue,
                                                std::function<bool(const double newValue)> callback) {
            m_NetworkTable->PutNumber(m_SubsystemName, defaultValue);
            m_NetworkTable->GetEntry(m_SubsystemName).AddListener(
                    [&](const nt::EntryNotification& notification) {
                        const auto newValue = notification.value->GetDouble();
                        const bool success = callback(newValue);
                        Log(Logger::LogLevel::k_Info,
                            Logger::Format("%s %s value %s to %d", success ? "Successfully set" : "Error in setting",
                                           FMT_STR(m_SubsystemName), FMT_STR(entryName), newValue));
                    }, NT_NOTIFY_UPDATE);
        }

        void Subsystem::Reset() {
            Log(lib::Logger::LogLevel::k_Info, "Reset");
            Unlock();
            OnReset();
        }

        void Subsystem::PostInitialize() {
            OnPostInitialize();
            Reset();
        }
    }
}
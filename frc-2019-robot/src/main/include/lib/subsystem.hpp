#pragma once

#include <command.hpp>

#include <lib/logger.hpp>

#include <memory>
#include <string>
#include <functional>

#include <networktables/NetworkTable.h>

#define SPACED_UPDATE_INTERVAL 5
#define DEFAULT_FREQUENCY 10

#define DEFAULT_INPUT_THRESHOLD 0.08
#define XBOX_360_STICK_INPUT_THRESHOLD 0.22
#define DEFAULT_VOLTAGE_COMPENSATION 11.0

namespace garage {
    class Robot;
    namespace lib {
        class Subsystem : public std::enable_shared_from_this<Subsystem> {
        protected:
            std::shared_ptr<Robot> m_Robot;
            std::shared_ptr<nt::NetworkTable> m_NetworkTable;
            Command m_LastCommand = {};
            bool m_IsLocked = false;
            unsigned long m_SequenceNumber = 0;
            std::string m_SubsystemName;

            virtual void AdvanceSequence();

            virtual void SpacedUpdate(Command& command) {}

            virtual bool ShouldUnlock(Command& command);

            virtual void UpdateUnlocked(Command& command) {}

            virtual void UpdateLocked() {}

            virtual void Update() {}

            virtual void ResetUnlock();

            void AddNetworkTableListener(const std::string& entryName, const double defaultValue,
                                         std::function<bool(const double newValue)> callback);

        public:
            Subsystem(std::shared_ptr<Robot>& robot, const std::string& subsystemName);

            void PostInitialize();

            virtual void OnPostInitialize() {}

            virtual void Reset();

            void Periodic();

            void Log(Logger::LogLevel logLevel, const std::string& log);

            void LogSample(Logger::LogLevel logLevel, const std::string& log, int frequency = DEFAULT_FREQUENCY);

            virtual void Lock();

            virtual void Unlock();

            bool IsLocked() {
                return m_IsLocked;
            }
        };
    }
}

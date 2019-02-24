#pragma once

#include <command.hpp>

#include <lib/logger.hpp>

#include <memory>
#include <string>

#define SPACED_UPDATE_INTERVAL 100
#define DEFAULT_FREQUENCY 10

#define DEFAULT_INPUT_THRESHOLD 0.075

namespace garage {
    class Robot;
    namespace lib {
        template<typename TSubsystem>
        class SubsystemController;

        class Subsystem : public std::enable_shared_from_this<Subsystem> {
        private:
            template<typename TSubsystem>
            friend SubsystemController;

        protected:
            std::shared_ptr<Robot> m_Robot;
            Command m_LastCommand = {};
            bool m_IsLocked = false;
            unsigned long m_SequenceNumber = 0;
            std::string m_SubsystemName;

            virtual void AdvanceSequence();

            virtual void SpacedUpdate(Command& command) {};

            virtual bool ShouldUnlock(Command& command);

            virtual void UpdateUnlocked(Command& command) {}

            virtual void UpdateLocked() {}

            virtual void Update() {};

            virtual void SetLastCommand(Command& command);

            virtual void OnLock() {};

            virtual void OnUnlock() {};

        public:
            Subsystem(std::shared_ptr<Robot>& robot, const std::string& subsystemName);

            virtual void TeleopInit() {};

            void Periodic();

            void Log(lib::LogLevel logLevel, std::string log);

            void LogSample(lib::LogLevel logLevel, std::string log, int frequency = DEFAULT_FREQUENCY);

            void Lock();

            void Unlock();

            bool IsLocked() {
                return m_IsLocked;
            }
        };
    }
}

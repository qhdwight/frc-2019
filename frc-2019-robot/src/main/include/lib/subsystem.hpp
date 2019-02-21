#pragma once

#include <command.hpp>

#include <lib/logger.hpp>

#include <memory>
#include <string>

#define SPACED_UPDATE_INTERVAL 100
#define DEFAULT_FREQUENCY 10

namespace garage {
    class Robot;
    namespace lib {
        class Subsystem {
        protected:
            std::shared_ptr<Robot> m_Robot;
            Command m_LastCommand = {};
            bool m_IsLocked = false;
            unsigned long m_SequenceNumber = 0;
            std::string m_SubsystemName;

            virtual void AdvanceSequence();

            virtual void SpacedUpdate(Command& command) {};

            virtual void ProcessCommand(Command& command);

            virtual void SetLastCommand(Command& command);

            virtual void Update() {};

        public:
            Subsystem(std::shared_ptr<Robot>& robot, std::string subsystemName);

            virtual void TeleopInit() {};

            void Periodic(Command& command);

            void Log(lib::LogLevel logLevel, std::string log);

            void LogSample(lib::LogLevel logLevel, std::string log, int frequency = DEFAULT_FREQUENCY);

            void Lock();

            void Unlock();

            bool IsLocked();
        };
    }
}

#pragma once

#include <command.hpp>

#include <lib/logger.hpp>

#include <memory>
#include <string>

#define SPACED_UPDATE_INTERVAL 100

namespace garage {
    class Robot;
    namespace lib {
        class Subsystem {
        protected:
            std::shared_ptr<Robot> m_Robot;
            Command m_LastCommand;
            bool m_IsLocked;
            unsigned long m_SequenceNumber;
            std::string m_SubsystemName;

        public:
            Subsystem(std::shared_ptr<Robot>& robot, std::string subsystemName);

            virtual void TeleopInit() {};

            virtual void Update(Command& command);

            virtual void SpacedUpdate();

            virtual void AdvanceSequence();

            virtual void ExecuteCommand(Command& command);

            virtual void SetLastCommand(Command& command);

            void Log(lib::LogLevel logLevel, std::string log);

            void Lock();

            void Unlock();
        };
    }
}

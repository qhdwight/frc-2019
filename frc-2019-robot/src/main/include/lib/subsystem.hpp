#pragma once

#include <command.hpp>

#include <memory>

namespace garage {
    class Robot;
    namespace lib {
        class Subsystem {
        protected:
            std::shared_ptr<Robot> m_Robot;
            Command m_LastCommand;
        public:
            Subsystem(std::shared_ptr<Robot>& robot);

            virtual void TeleopInit() {};

            virtual void ExecuteCommand(Command& command);
        };
    }
}

#pragma once

#include <command.hpp>

#include <memory>

namespace garage {
    class Robot;
    namespace lib {
        class Subsystem : public std::enable_shared_from_this<Subsystem> {
        protected:
            std::shared_ptr<Robot> m_Robot;
        public:
            Subsystem(std::shared_ptr<Robot>& robot);

            virtual void Initialize() {};

            virtual void TeleopInit() {};

            virtual void ExecuteCommand(Command& command) {};
        };
    }
}

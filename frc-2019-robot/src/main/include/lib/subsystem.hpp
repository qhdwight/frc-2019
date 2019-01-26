#pragma once

#include <command.hpp>

namespace garage {
    namespace lib {
        class Subsystem {
        public:
            Subsystem();

            virtual void Initialize() {};

            virtual void ExecuteCommand(Command& command) {};
        };
    }
}
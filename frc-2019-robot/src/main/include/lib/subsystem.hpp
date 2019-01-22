#pragma once

#include <command.hpp>

namespace garage {
    namespace lib {
        class Subsystem {
        public:
            Subsystem() = default;
            virtual void ExecuteCommand(Command& command) {};
        };
    }
}
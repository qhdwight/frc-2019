#pragma once

#include <command.hpp>

#include <memory>

namespace garage {
    namespace lib {
        template<typename TSubsystem>
        class SubsystemController {
        protected:
            std::shared_ptr<TSubsystem> m_Subsystem;

        public:
            SubsystemController(std::shared_ptr<TSubsystem>& flipper) : m_Subsystem(flipper) {};

            virtual void Control(Command& command) {};
        };
    }
}
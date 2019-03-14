#pragma once

#include <lib/routine.hpp>
#include <lib/subsystem.hpp>

namespace garage {
    namespace lib {
        template<typename TSubsystem>
        class SubsystemRoutine : public lib::Routine {
        protected:
            std::shared_ptr<TSubsystem> m_Subsystem;

        public:
            SubsystemRoutine(std::shared_ptr<Robot> robot, std::shared_ptr<TSubsystem> subsystem, const std::string& name)
                : Routine(robot, name), m_Subsystem(subsystem) { }
        };
    }
}
#pragma once

#include <robot.hpp>

#include <lib/routine.hpp>
#include <lib/subsystem.hpp>

namespace garage {
    namespace lib {
        template<typename TSubsystem>
        class SubsystemRoutine : public lib::Routine {
        protected:
            std::shared_ptr<TSubsystem> m_Subsystem;

        public:
            SubsystemRoutine(std::shared_ptr<Robot> robot, const std::string& name)
                : Routine(robot, name), m_Subsystem(robot->GetSubsystem<TSubsystem>()) { }

            bool ShouldTerminateBasedOnUnlock(std::shared_ptr<Subsystem> subsystem) override {
                return subsystem == m_Subsystem;
            }
        };
    }
}
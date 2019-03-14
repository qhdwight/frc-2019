#pragma once

#include <lib/routine.hpp>
#include <lib/subsystem_routine.hpp>

namespace garage {
    class Flipper;

    class SetFlipperServoRoutine : public lib::SubsystemRoutine<Flipper> {
    protected:
        bool m_ShouldLock = false;

        void Start() override;

    public:
        SetFlipperServoRoutine(std::shared_ptr<Robot> robot, bool shouldLock, const std::string& name = "Set Flipper Servo");
    };
}
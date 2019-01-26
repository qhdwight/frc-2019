#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#include <lib/subsystem.hpp>

namespace garage {
    class BallIntake : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::VictorSPX m_LeftIntake{6}, m_RightIntake{7};
    public:
        void Initialize() override;

        void ExecuteCommand(Command& command) override;
    };
}

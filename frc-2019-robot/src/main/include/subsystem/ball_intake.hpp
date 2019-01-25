#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <lib/subsystem.hpp>

namespace garage {
    class BallIntake : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_LeftIntakeSPX{6}, m_RightIntakeSPX{7};
    public:
        BallIntake();

        void ExecuteCommand(Command& command) override;
    };
}

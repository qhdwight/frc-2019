#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#include <lib/subsystem.hpp>

namespace garage {
    class Elevator : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::TalonSRX m_ElevatorMaster{0};
    public:
        void Initialize() override;

        void ExecuteCommand(Command& command) override;
    };
}

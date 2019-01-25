#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <lib/subsystem.hpp>

namespace garage {
    class Elevator : public lib::Subsystem {
    private:
//        ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_IntakeSPX{5};
    public:
        void ExecuteCommand(Command& command) override;
    };
}

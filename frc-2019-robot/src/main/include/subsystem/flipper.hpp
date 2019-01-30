#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

namespace garage {
    class Flipper : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::TalonSRX m_Flipper{FLIPPER};
    public:
        Flipper(std::shared_ptr<Robot>& robot) : Subsystem(robot) {}

        void Initialize() override;

        void ExecuteCommand(Command& command) override;
    };
}

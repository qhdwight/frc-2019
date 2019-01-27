#pragma once

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

namespace garage {
    class Elevator : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::TalonSRX m_ElevatorMaster{0};
    public:
        Elevator(std::shared_ptr<Robot>& robot) : Subsystem(robot) {}

        void Initialize() override;

        void ExecuteCommand(Command& command) override;
    };
}

#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

namespace garage {
    class BallIntake : public lib::Subsystem {
    private:
        double m_LastOpenLoopRamp = 0.0;
        ctre::phoenix::motorcontrol::can::TalonSRX m_RightIntake{BALL_INTAKE_MASTER}, m_LeftIntake{BALL_INTAKE_SLAVE};
    public:
        BallIntake(std::shared_ptr<Robot>& robot);

        void ExecuteCommand(Command& command) override;
    };
}

#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#define OUTPUT_PROPORTION_INTAKING 0.25
#define OUTPUT_PROPORTION_EXPELLING 0.8

#define RAMP_INTAKING 0.2
#define RAMP_EXPELLING 0.05

namespace garage {
    class BallIntake : public lib::Subsystem {
    private:
        double m_Output = 0.0, m_LastOpenLoopRamp = 0.0;
        ctre::phoenix::motorcontrol::can::TalonSRX m_RightIntake{BALL_INTAKE_MASTER}, m_LeftIntake{BALL_INTAKE_SLAVE};
        
    protected:
        void ProcessCommand(Command& command) override;

        void Update() override;

    public:
        BallIntake(std::shared_ptr<Robot>& robot);
    };
}

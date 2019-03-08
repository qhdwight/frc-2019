#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#define OUTPUT_PROPORTION_INTAKING 0.25
#define OUTPUT_PROPORTION_EXPELLING 0.8

#define RAMP_INTAKING 0.2
#define RAMP_EXPELLING 0.05

#define HAS_BALL_STALL_CURRENT 5.0
#define HAS_BALL_COUNTS_REQUIRED 5

namespace garage {
    enum class IntakeMode {
        k_Idle, k_Intaking, k_Expelling
    };

    class BallIntake : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::TalonSRX m_RightIntake{BALL_INTAKE_MASTER}, m_LeftIntake{BALL_INTAKE_SLAVE};

        void SetOutput(double output);

        void ConfigOpenLoopRamp(double ramp);

    protected:

        void UpdateUnlocked(Command& command) override;

    public:
        BallIntake(std::shared_ptr<Robot>& robot);

        void SetMode(IntakeMode intakeMode, double strength = 0.0);

        bool GetHasBall();
    };
}

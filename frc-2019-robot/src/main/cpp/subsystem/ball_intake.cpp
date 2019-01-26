#include <subsystem/ball_intake.hpp>

namespace garage {
    void BallIntake::Initialize() {
        m_LeftIntake.ConfigFactoryDefault();
        m_RightIntake.ConfigFactoryDefault();
        m_LeftIntake.ConfigOpenloopRamp(1.0);
        m_RightIntake.ConfigOpenloopRamp(1.0);
        m_LeftIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_RightIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    }

    void BallIntake::ExecuteCommand(Command& command) {
        const double ballIntake = command.ballIntake;
        m_LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ballIntake);
        m_RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ballIntake);
    }
}

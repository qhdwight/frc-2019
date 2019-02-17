#include <subsystem/ball_intake.hpp>

#include <robot.hpp>

namespace garage {
    BallIntake::BallIntake(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
        m_LeftIntake.ConfigFactoryDefault();
        m_RightIntake.ConfigFactoryDefault();
        m_LeftIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_RightIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    }

    void BallIntake::ExecuteCommand(Command& command) {
        m_Robot->GetNetworkTable()->PutNumber("Ball Intake/Current", m_RightIntake.GetOutputCurrent());
        const double ballIntake = command.ballIntake;
        const bool intaking = ballIntake < 0.0;
        double multiplier;
        if (intaking) {
            multiplier = 0.25;
            m_LeftIntake.ConfigOpenloopRamp(0.2);
            m_RightIntake.ConfigOpenloopRamp(0.2);
        } else {
            multiplier = 0.8;
            m_LeftIntake.ConfigOpenloopRamp(0.05);
            m_RightIntake.ConfigOpenloopRamp(0.05);
        }
        m_LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ballIntake * multiplier);
        m_RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ballIntake * multiplier);
        Subsystem::ExecuteCommand(command);
    }
}

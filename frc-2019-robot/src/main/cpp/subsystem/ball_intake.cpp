#include <subsystem/ball_intake.hpp>

#include <robot.hpp>

namespace garage {
    BallIntake::BallIntake(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Ball Intake") {
        m_LeftIntake.ConfigFactoryDefault();
        m_RightIntake.ConfigFactoryDefault();
        m_LeftIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_RightIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    }

    void BallIntake::ExecuteCommand(Command& command) {
//        m_Robot->GetNetworkTable()->PutNumber("Ball Intake/Current", m_RightIntake.GetOutputCurrent());
        const double ballIntake = command.ballIntake;
        const bool intaking = ballIntake < 0.0;
        double multiplier, openLoopRamp;
        if (intaking) {
            multiplier = 0.25;
            openLoopRamp = 0.2;
        } else {
            multiplier = 0.8;
            openLoopRamp = 0.05;
        }
        if (openLoopRamp != m_LastOpenLoopRamp) {
            m_LeftIntake.ConfigOpenloopRamp(openLoopRamp);
            m_RightIntake.ConfigOpenloopRamp(openLoopRamp);
            m_LastOpenLoopRamp = openLoopRamp;
        }
        m_LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ballIntake * multiplier);
        m_RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ballIntake * multiplier);
    }
}

#include <subsystem/ball_intake.hpp>

#include <robot.hpp>

namespace garage {
    BallIntake::BallIntake(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Ball Intake") {
        m_LeftIntake.ConfigFactoryDefault();
        m_RightIntake.ConfigFactoryDefault();
        m_LeftIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_RightIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    }

    void BallIntake::ProcessCommand(Command& command) {
//        m_Robot->GetNetworkTable()->PutNumber("Ball Intake/Current", m_RightIntake.GetOutputCurrent());
        m_Output = math::threshold(command.ballIntake, 0.05);
    }

    void BallIntake::Update() {
        const auto isIntaking = m_Output < 0.0;
        double outputProportion, openLoopRamp;
        if (isIntaking) {
            outputProportion = OUTPUT_PROPORTION_INTAKING;
            openLoopRamp = RAMP_INTAKING;
        } else {
            outputProportion = OUTPUT_PROPORTION_EXPELLING;
            openLoopRamp = RAMP_EXPELLING;
        }
        if (openLoopRamp != m_LastOpenLoopRamp) {
            m_LeftIntake.ConfigOpenloopRamp(openLoopRamp);
            m_RightIntake.ConfigOpenloopRamp(openLoopRamp);
            m_LastOpenLoopRamp = openLoopRamp;
        }
        m_LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_Output * outputProportion);
        m_RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_Output * outputProportion);
    }
}

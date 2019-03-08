#include <subsystem/ball_intake.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

namespace garage {
    BallIntake::BallIntake(std::shared_ptr<Robot>& robot) : lib::Subsystem(robot, "Ball Intake") {
        m_LeftIntake.ConfigFactoryDefault();
        m_RightIntake.ConfigFactoryDefault();
        m_LeftIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_RightIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    }

    void BallIntake::OnReset() {
        SetOutput(0.0);
        m_LastOutput = 0.0;
        m_HasBallCount = 0;
    }

    void BallIntake::UpdateUnlocked(Command& command) {
//        m_NetworkTable->PutNumber("Ball Intake/Current", m_RightIntake.GetOutputCurrent());
        double input = command.ballIntake, absoluteInput = math::absolute(input);
        if (absoluteInput > 0.5) input = math::sign(input) * 1.0;
        IntakeMode intakeMode;
        if (input > DEFAULT_INPUT_THRESHOLD) intakeMode = IntakeMode::k_Expelling;
        else if (input < -DEFAULT_INPUT_THRESHOLD) intakeMode = IntakeMode::k_Intaking;
        else intakeMode = IntakeMode::k_Idle;
        SetMode(intakeMode, absoluteInput);
    }

    bool BallIntake::GetHasBall() {
        if (m_RightIntake.GetOutputCurrent() > HAS_BALL_STALL_CURRENT)
            m_HasBallCount++;
        else if (m_HasBallCount > 0)
            m_HasBallCount = 0;
        return m_HasBallCount > HAS_BALL_COUNTS_REQUIRED;
    }

    void BallIntake::SetOutput(double output) {
        if (m_LastOutput != output) {
            m_RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output);
            m_LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output);
            m_LastOutput = output;
        }
    }

    void BallIntake::ConfigOpenLoopRamp(double ramp) {
        static double s_LastRamp = 0.0;
        if (s_LastRamp != ramp) {
            auto leftError = m_LeftIntake.ConfigOpenloopRamp(ramp), rightError = m_RightIntake.ConfigOpenloopRamp(ramp);
            if (leftError == ctre::phoenix::OK && rightError == ctre::phoenix::OK)
                s_LastRamp = ramp;
            else
                Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("Left Error: %d, Right Error: %d", leftError, rightError));
        }
    }

    void BallIntake::SetMode(IntakeMode intakeMode, double strength) {
        switch (intakeMode) {
            case IntakeMode::k_Idle: {
                SetOutput(0.0);
                break;
            }
            case IntakeMode::k_Intaking: {
                SetOutput(OUTPUT_PROPORTION_INTAKING * strength);
                ConfigOpenLoopRamp(RAMP_INTAKING);
                break;
            }
            case IntakeMode::k_Expelling: {
                SetOutput(OUTPUT_PROPORTION_EXPELLING * strength);
                ConfigOpenLoopRamp(RAMP_EXPELLING);
                break;
            }
        }
    }
}

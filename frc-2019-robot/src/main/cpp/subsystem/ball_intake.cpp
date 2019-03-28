#include <subsystem/ball_intake.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

namespace garage {
    BallIntake::BallIntake(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Ball Intake") {
        m_LeftIntake.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_RightIntake.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_LeftIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_RightIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_LeftIntake.ConfigVoltageCompSaturation(DEFAULT_VOLTAGE_COMPENSATION, CONFIG_TIMEOUT);
        m_RightIntake.ConfigVoltageCompSaturation(DEFAULT_VOLTAGE_COMPENSATION, CONFIG_TIMEOUT);
        m_LeftIntake.EnableVoltageCompensation(true);
        m_RightIntake.EnableVoltageCompensation(true);
        m_RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        m_LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        SetOutput(0.0);
        ConfigOpenLoopRamp(0.15);
    }

    void BallIntake::Reset() {
        Subsystem::Reset();
        SetOutput(0.0);
    }

    bool BallIntake::ShouldUnlock(Command& command) {
        return std::fabs(command.ballIntake) > DEFAULT_INPUT_THRESHOLD;
    }

    void BallIntake::UpdateUnlocked(Command& command) {
        double input = command.ballIntake, absoluteInput = std::fabs(input);
        if (absoluteInput > 0.5) input = math::sign(input) * 1.0;
        if (input > 0.0) {
            SetIntakeMode(IntakeMode::k_Intaking, absoluteInput);
        } else if (input < 0.0) {
            SetIntakeMode(IntakeMode::k_Expelling, absoluteInput);
        } else {
            SetOutput(0.0);
        }
    }

    void BallIntake::SpacedUpdate(Command& command) {
        const double outputCurrent = m_RightIntake.GetOutputCurrent();
        m_NetworkTable->PutNumber("Current", outputCurrent);
        if (outputCurrent > HAS_BALL_STALL_CURRENT) {
            m_HasBallCount++;
        } else if (m_HasBallCount > 0) {
            m_HasBallCount = 0;
        }
    }

    bool BallIntake::HasBall() {
        return m_HasBallCount > HAS_BALL_COUNTS_REQUIRED;
    }

    void BallIntake::SetOutput(double output) {
        if (m_Robot->ShouldOutput()) {
            m_RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output);
            m_LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output);
            m_LastOutput = output;
        }
    }

    void BallIntake::ConfigOpenLoopRamp(double ramp) {
        if (m_LastOpenLoopRamp != ramp) {
            auto leftError = m_LeftIntake.ConfigOpenloopRamp(ramp), rightError = m_RightIntake.ConfigOpenloopRamp(ramp);
            if (leftError == ctre::phoenix::OK && rightError == ctre::phoenix::OK) {
                m_LastOpenLoopRamp = ramp;
            } else {
                Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("Left Error: %d, Right Error: %d", leftError, rightError));
            }
        }
    }

    void BallIntake::SetIntakeMode(IntakeMode intakeMode, double strength) {
        switch (intakeMode) {
            case IntakeMode::k_Intaking: {
                SetOutput(OUTPUT_PROPORTION_INTAKING * strength * -1);
//                ConfigOpenLoopRamp(RAMP_INTAKING);
                break;
            }
            case IntakeMode::k_Expelling: {
                SetOutput(OUTPUT_PROPORTION_EXPELLING * strength);
//                ConfigOpenLoopRamp(RAMP_EXPELLING);
                break;
            }
        }
    }

    void BallIntake::Expell(double strength) {
        Lock();
        SetIntakeMode(IntakeMode::k_Expelling, strength);
    }

    void BallIntake::Intake(double strength) {
        Lock();
        SetIntakeMode(IntakeMode::k_Intaking, strength);
    }

    void BallIntake::Stop() {
        Lock();
        SetOutput(0.0);
    }
}

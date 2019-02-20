#include <subsystem/flipper.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <lib/logger.hpp>

namespace garage {
    Flipper::Flipper(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Flipper") {
        m_Flipper.RestoreFactoryDefaults();
        m_Flipper.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_Flipper.SetOpenLoopRampRate(0.2);
        m_Flipper.SetClosedLoopRampRate(0.2);
        m_FlipperController.SetP(FLIPPER_P);
        m_FlipperController.SetI(FLIPPER_I);
        m_FlipperController.SetD(FLIPPER_D);
        m_FlipperController.SetIZone(FLIPPER_I_ZONE);
        m_FlipperController.SetFF(FLIPPER_FF);
        m_FlipperController.SetOutputRange(-1.0, 1.0);
        m_FlipperController.SetSmartMotionMaxVelocity(FLIPPER_VELOCITY);
        m_FlipperController.SetSmartMotionMinOutputVelocity(0.0);
        m_FlipperController.SetSmartMotionMaxAccel(FLIPPER_ACCELERATION);
        m_FlipperController.SetSmartMotionAllowedClosedLoopError(FLIPPER_ALLOWABLE_ERROR);
        m_LimitSwitch.EnableLimitSwitch(false);
    }

    void Flipper::TeleopInit() {

    }

    void Flipper::ExecuteCommand(Command& command) {
        const bool isLimitSwitchClosed = m_LimitSwitch.Get();
        if (isLimitSwitchClosed && m_FirstLimitSwitchHit) {
            auto error = m_Encoder.SetPosition(0.0);
            if (error == rev::CANError::kOK) {
                Log(lib::LogLevel::k_Info, "Limit switch hit and encoder reset");
                m_FirstLimitSwitchHit = false;
            } else {
                Log(lib::LogLevel::k_Error, "CAN Error: " + std::to_string(static_cast<int>(error)));
            }
        }
        if (!isLimitSwitchClosed) {
            m_FirstLimitSwitchHit = true;
        }
        const auto encoderPosition = m_Encoder.GetPosition();
        const auto faults = m_Flipper.GetStickyFaults();
        m_Flipper.ClearFaults();
        double setPoint = command.flipper;
        const bool inMiddle = encoderPosition > FLIPPER_LOWER && encoderPosition < FLIPPER_UPPER;
        bool wantingToGoOtherWay = false;
        if (encoderPosition < FLIPPER_LOWER && setPoint > FLIPPER_LOWER) wantingToGoOtherWay = true;
        if (encoderPosition > FLIPPER_UPPER && setPoint < FLIPPER_UPPER) wantingToGoOtherWay = true;
        if (inMiddle || wantingToGoOtherWay) {
            setPoint = math::clamp(setPoint, FLIPPER_LOWER, FLIPPER_UPPER);
            if (setPoint != m_LastSetPoint) {
                auto error = m_FlipperController.SetReference(setPoint, rev::ControlType::kSmartMotion);
                if (error == rev::CANError::kOK) {
                    Log(lib::LogLevel::k_Info, "Setting set point to: " + std::to_string(setPoint));
                    m_LastSetPoint = setPoint;
                } else {
                    Log(lib::LogLevel::k_Error, "CAN Error: " + std::to_string(static_cast<int>(error)));
                }
            }
        } else {
            LogSample(lib::LogLevel::k_Info, "Not doing anything");
            m_Flipper.Set(0.0);
        }
        if (faults != 0)
            LogSample(lib::LogLevel::k_Error, "Stick Fault Error: " + std::to_string(faults));
        LogSample(lib::LogLevel::k_Info, "Wanted Position: " + std::to_string(setPoint) + ", Output: " + std::to_string(m_Flipper.GetAppliedOutput()) + ", Limit Switch: " + std::to_string(isLimitSwitchClosed) + ", Encoder position: " + std::to_string(encoderPosition) + ", Encoder Velocity: " + std::to_string(m_Encoder.GetVelocity()) + ", Faults: " + std::to_string(faults));

//        const double output = math::threshold(m_LastCommand.ballIntake, 0.05) * 0.35;
//        m_Flipper.Set(output);
    }
}
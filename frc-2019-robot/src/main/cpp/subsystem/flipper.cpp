#include <subsystem/flipper.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <lib/logger.hpp>

namespace garage {
    Flipper::Flipper(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Flipper") {
        m_Flipper.RestoreFactoryDefaults();
        m_Flipper.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_Flipper.SetOpenLoopRampRate(0.2);
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

    void Flipper::UpdateUnlocked(Command& command) {
        m_SetPoint += math::threshold(command.flipper, DEFAULT_INPUT_THRESHOLD);
        m_SetPoint = math::clamp(m_SetPoint, FLIPPER_LOWER, FLIPPER_UPPER);
    }

    void Flipper::Update() {
        m_IsLimitSwitchDown = m_LimitSwitch.Get();
        static bool s_FirstLimitSwitchHit = true;
        if (m_IsLimitSwitchDown && s_FirstLimitSwitchHit) {
            auto error = m_Encoder.SetPosition(0.0);
            if (error == rev::CANError::kOK) {
                Log(lib::LogLevel::k_Info, "Limit switch hit and encoder reset");
                s_FirstLimitSwitchHit = false;
            } else {
                Log(lib::LogLevel::k_Error, m_Robot->GetLogger()->Format("CAN Error: %d", error));
            }
        }
        if (!m_IsLimitSwitchDown)
            s_FirstLimitSwitchHit = true;
        m_EncoderPosition = m_Encoder.GetPosition();
        const uint16_t faults = m_Flipper.GetStickyFaults();
        m_Flipper.ClearFaults();
        const bool inMiddle = m_EncoderPosition > FLIPPER_SET_POINT_LOWER && m_EncoderPosition < FLIPPER_SET_POINT_UPPER;
        bool wantingToGoOtherWay = false;
        if ((m_EncoderPosition < FLIPPER_SET_POINT_LOWER && m_SetPoint > FLIPPER_SET_POINT_LOWER) ||
            (m_EncoderPosition > FLIPPER_SET_POINT_UPPER && m_SetPoint < FLIPPER_SET_POINT_UPPER))
            wantingToGoOtherWay = true;
        if (inMiddle || wantingToGoOtherWay) {
            const double clampedSetPoint = math::clamp(m_SetPoint, FLIPPER_SET_POINT_LOWER, FLIPPER_SET_POINT_UPPER);
            if (clampedSetPoint != m_LastSetPoint) {
                auto error = m_FlipperController.SetReference(clampedSetPoint, rev::ControlType::kSmartMotion);
                if (error == rev::CANError::kOK) {
                    Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format("Setting set point to: %d", clampedSetPoint));
                    m_LastSetPoint = clampedSetPoint;
                } else {
                    Log(lib::LogLevel::k_Error, m_Robot->GetLogger()->Format("CAN Error: %d", error));
                }
            }
        } else {
            LogSample(lib::LogLevel::k_Info, "Not doing anything");
            m_Flipper.Set(0.0);
        }
        if (faults != 0)
            Log(lib::LogLevel::k_Error, m_Robot->GetLogger()->Format("Stick Fault Error: %d", faults));
//        const double output = math::threshold(m_LastCommand.ballIntake, DEFAULT_INPUT_THRESHOLD) * 0.35;
//        m_Flipper.Set(output);
    }

    void Flipper::SpacedUpdate(Command& command) {
        Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format(
                "Wanted Position: %d, Output: %f, Encoder Position: %f, Encoder Velocity: %f, Limit Switch: %d, Faults: %d",
                m_SetPoint, m_Flipper.GetAppliedOutput(), m_IsLimitSwitchDown, m_EncoderPosition, m_Encoder.GetVelocity()));
    }

    void Flipper::SetRawOutput(double output) {

    }

    void Flipper::SetSetPoint(double setPoint) {

    }

    void Flipper::SetOutput(FlipperControlMode flipperControlMode, double output, bool forceSet) {
        switch (flipperControlMode) {
            case FlipperControlMode::k_Manual: {
                static double s_LastOutput = 0.0;
                if (forceSet || output != s_LastOutput) {
                    m_Flipper.Set(output);
                    s_LastOutput = output;
                }
                break;
            }
            case FlipperControlMode::k_SetPoint: {
                static double s_LastWantedPosition = 0.0;
                if (forceSet || output != s_LastWantedPosition) {
                    m_FlipperController.SetReference(output, rev::ControlType::kSmartMotion);
                    s_LastWantedPosition = output;
                }
                break;
            }
        }
    }
}

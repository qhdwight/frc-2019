#include <subsystem/flipper.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <lib/logger.hpp>

namespace garage {
    Flipper::Flipper(std::shared_ptr<Robot>& robot) : lib::ControllableSubsystem<Flipper>(robot, "Flipper") {
        m_FlipperMaster.RestoreFactoryDefaults();
        m_FlipperMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_FlipperMaster.SetOpenLoopRampRate(FLIPPER_CLOSED_LOOP_RAMP);
        m_FlipperController.SetP(FLIPPER_P, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetI(FLIPPER_I, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetD(FLIPPER_D, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetIZone(FLIPPER_I_ZONE, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetFF(FLIPPER_FF, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetOutputRange(-1.0, 1.0, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetSmartMotionMaxVelocity(FLIPPER_VELOCITY, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetSmartMotionMinOutputVelocity(0.0, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetSmartMotionMaxAccel(FLIPPER_ACCELERATION, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetSmartMotionAllowedClosedLoopError(FLIPPER_ALLOWABLE_ERROR, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetSmartMotionAccelStrategy(rev::CANPIDController::AccelStrategy::kSCurve, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperMaster.EnableVoltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        m_LimitSwitch.EnableLimitSwitch(false);
        m_FlipperMaster.Set(0.0);
    }

    void Flipper::OnPostInitialize() {
        auto flipper = std::weak_ptr<Flipper>(shared_from_this());
        AddController(m_RawController = std::make_shared<RawFlipperController>(flipper));
        AddController(m_SetPointController = std::make_shared<SetPointFlipperController>(flipper));
        SetUnlockedController(m_RawController);
    }

    void Flipper::Update() {
        m_IsLimitSwitchDown = m_LimitSwitch.Get();
        if (m_IsLimitSwitchDown && m_FirstLimitSwitchHit) {
            auto error = m_Encoder.SetPosition(0.0);
            if (error == rev::CANError::kOK) {
                Log(lib::Logger::LogLevel::k_Info, "Limit switch hit and encoder reset");
                m_FirstLimitSwitchHit = false;
            } else {
                Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("CAN Error: %d", error));
            }
        }
        if (!m_IsLimitSwitchDown) {
            m_FirstLimitSwitchHit = true;
        }
        m_EncoderPosition = m_Encoder.GetPosition();
        m_EncoderVelocity = m_Encoder.GetVelocity();
        const uint16_t faults = m_FlipperMaster.GetStickyFaults();
        if (faults != 0) {
            Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("Stick Fault Error: %d", faults));
            m_FlipperMaster.ClearFaults();
        }
//        const double output = math::threshold(m_LastCommand.ballIntake, DEFAULT_INPUT_THRESHOLD) * 0.35;
//        m_Flipper.Set(output);
    }

    void Flipper::SpacedUpdate(Command& command) {
        Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format(
                "Output: %f, Encoder Position: %f, Encoder Velocity: %f, Limit Switch: %s",
                m_FlipperMaster.GetAppliedOutput(), m_EncoderPosition, m_EncoderVelocity, m_IsLimitSwitchDown ? "true" : "false"));
    }

    void Flipper::SetRawOutput(double output) {
        SetController(m_RawController);
    }

    void Flipper::SetSetPoint(double setPoint) {
        SetController(m_SetPointController);
        m_SetPointController->SetSetPoint(setPoint);
    }

    void Flipper::SetAngle(double angle) {
        SetSetPoint(AngleToRawSetPoint(angle));
    }

    void Flipper::Stow() {
        SetAngle(90.0);
    }

    double Flipper::RawSetPointToAngle(double setPoint) {
        return math::map(setPoint, FLIPPER_LOWER, FLIPPER_UPPER, FLIPPER_LOWER_ANGLE, FLIPPER_UPPER_ANGLE);
    }

    double Flipper::AngleToRawSetPoint(double angle) {
        return math::map(angle, FLIPPER_LOWER_ANGLE, FLIPPER_UPPER_ANGLE, FLIPPER_LOWER, FLIPPER_UPPER);
    }

    /* =============================================================== Controllers ===============================================================

       =========================================================================================================================================== */

    void RawFlipperController::ProcessCommand(Command& command) {
        m_Input = math::threshold(command.flipper, DEFAULT_INPUT_THRESHOLD);
        m_Output = m_Input * 0.25;
    }

    void RawFlipperController::Control() {
        auto flipper = m_Subsystem.lock();
        if (flipper->m_Robot->ShouldOutputMotors()) {
            flipper->m_FlipperMaster.Set(m_Output);
        }
    }

    void RawFlipperController::Reset() {
        m_Input = 0.0;
        m_Output = 0.0;
    }

    void SetPointFlipperController::ProcessCommand(garage::Command& command) {
        m_SetPoint += math::threshold(command.flipper, DEFAULT_INPUT_THRESHOLD);
        m_SetPoint = math::clamp(m_SetPoint, FLIPPER_LOWER, FLIPPER_UPPER);
    }

    void SetPointFlipperController::Control() {
        auto flipper = m_Subsystem.lock();
        double encoderPosition = flipper->m_EncoderPosition;
        const bool inMiddle = encoderPosition > FLIPPER_SET_POINT_LOWER && encoderPosition < FLIPPER_SET_POINT_UPPER;
        bool wantingToGoOtherWay = false;
        if ((encoderPosition < FLIPPER_SET_POINT_LOWER && m_SetPoint > FLIPPER_SET_POINT_LOWER) ||
            (encoderPosition > FLIPPER_SET_POINT_UPPER && m_SetPoint < FLIPPER_SET_POINT_UPPER))
            wantingToGoOtherWay = true;
        if (inMiddle || wantingToGoOtherWay) {
            const double
                    clampedSetPoint = math::clamp(m_SetPoint, FLIPPER_SET_POINT_LOWER, FLIPPER_SET_POINT_UPPER),
                    angle = flipper->RawSetPointToAngle(encoderPosition),
                    feedForward = std::cos(r2d(angle)) * FLIPPER_ANGLE_FF;
            if (flipper->m_Robot->ShouldOutputMotors()) {
                auto error = flipper->m_FlipperController.SetReference(clampedSetPoint, rev::ControlType::kSmartMotion,
                                                                       FLIPPER_SMART_MOTION_PID_SLOT, feedForward);
                if (error == rev::CANError::kOK) {
                    Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("Setting set point to: %d", clampedSetPoint));
                } else {
                    Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("CAN Error: %d", error));
                }
            }
        } else {
            flipper->LogSample(lib::Logger::LogLevel::k_Info, "Not doing anything");
            if (flipper->m_Robot->ShouldOutputMotors()) {
                flipper->m_FlipperMaster.Set(0.0);
            }
        }
    }

    void SetPointFlipperController::Reset() {
        m_SetPoint = 0.0;
    }
}

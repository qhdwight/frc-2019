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
        m_FlipperController.SetIMaxAccum(FLIPPER_MAX_ACCUM, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetFF(FLIPPER_FF, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetOutputRange(-1.0, 1.0, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetSmartMotionMaxVelocity(FLIPPER_VELOCITY, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetSmartMotionMinOutputVelocity(0.0, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetSmartMotionMaxAccel(FLIPPER_ACCELERATION, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperController.SetSmartMotionAllowedClosedLoopError(FLIPPER_ALLOWABLE_ERROR, FLIPPER_SMART_MOTION_PID_SLOT);
        // TODO yay or nay with s-curve?
        m_FlipperController.SetSmartMotionAccelStrategy(rev::CANPIDController::AccelStrategy::kSCurve, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperMaster.EnableVoltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        m_LimitSwitch.EnableLimitSwitch(false);
        m_FlipperMaster.Set(0.0);
    }

    void Flipper::OnPostInitialize() {
        auto flipper = std::weak_ptr<Flipper>(shared_from_this());
        AddController(m_RawController = std::make_shared<RawFlipperController>(flipper));
        AddController(m_SetPointController = std::make_shared<SetPointFlipperController>(flipper));
        AddController(m_VelocityController = std::make_shared<VelocityFlipperController>(flipper));
        SetUnlockedController(m_VelocityController);
        AddNetworkTableListener("Angle FF", FLIPPER_ANGLE_FF, [this](const double angleFF) {
            m_AngleFeedForward = angleFF;
            return true;
        });
        AddNetworkTableListener("P", FLIPPER_P, [this](const double p) {
            auto error = m_FlipperController.SetP(p, FLIPPER_SMART_MOTION_PID_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("I", FLIPPER_I, [this](const double i) {
            auto error = m_FlipperController.SetI(i, FLIPPER_SMART_MOTION_PID_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("I Zone", FLIPPER_I_ZONE, [this](const double iZone) {
            auto error = m_FlipperController.SetIZone(iZone, FLIPPER_SMART_MOTION_PID_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("Max Accum", FLIPPER_MAX_ACCUM, [this](const double maxAccum) {
            auto error = m_FlipperController.SetIMaxAccum(maxAccum, FLIPPER_SMART_MOTION_PID_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("D", FLIPPER_D, [this](const double d) {
            auto error = m_FlipperController.SetD(d, FLIPPER_SMART_MOTION_PID_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("FF", FLIPPER_FF, [this](const double ff) {
            auto error = m_FlipperController.SetFF(ff, FLIPPER_SMART_MOTION_PID_SLOT);
            return error == rev::CANError::kOK;
        });
    }

    void Flipper::Reset() {
        ControllableSubsystem::Reset();
        m_LockServoOutput = LOCK_SERVO_LOWER;
        m_CameraServoOutput = CAMERA_SERVO_LOWER;
    }

    bool Flipper::ShouldUnlock(Command& command) {
        return std::fabs(command.flipper) > DEFAULT_INPUT_THRESHOLD && !command.offTheBooksModeEnabled;
    }

    void Flipper::UpdateUnlocked(Command& command) {
        ControllableSubsystem::UpdateUnlocked(command);
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
        m_Angle = RawSetPointToAngle(m_EncoderPosition);
        const uint16_t faults = m_FlipperMaster.GetStickyFaults();
        if (faults != 0) {
            Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("Stick Fault Error: %d", faults));
            m_FlipperMaster.ClearFaults();
        }
        // TODO check
        m_CameraServoOutput = static_cast<uint16_t>(m_Angle > 90.0 ? CAMERA_SERVO_LOWER : CAMERA_SERVO_UPPER);
        m_CameraServo.SetRaw(m_CameraServoOutput);
        m_LockServo.SetAngle(m_LockServoOutput);
    }

    void Flipper::SpacedUpdate(Command& command) {
        Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format(
                "Output: %f, Angle: %f, Encoder Position: %f, Encoder Velocity: %f, Limit Switch: %s",
                m_FlipperMaster.GetAppliedOutput(), m_Angle, m_EncoderPosition, m_EncoderVelocity, m_IsLimitSwitchDown ? "true" : "false"));
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

    void Flipper::LockServo() {
        Lock();
        m_LockServoOutput = LOCK_SERVO_UPPER;
    }

    void Flipper::UnlockServo() {
        m_LockServoOutput = LOCK_SERVO_LOWER;
    }

    double Flipper::RawSetPointToAngle(double setPoint) {
        return math::map(setPoint, FLIPPER_LOWER, FLIPPER_UPPER, FLIPPER_LOWER_ANGLE, FLIPPER_UPPER_ANGLE);
    }

    double Flipper::AngleToRawSetPoint(double angle) {
        return math::map(angle, FLIPPER_LOWER_ANGLE, FLIPPER_UPPER_ANGLE, FLIPPER_LOWER, FLIPPER_UPPER);
    }

    bool Flipper::WithinAngle(double angle) {
        return math::withinRange(m_Angle, angle, 1.5);
    }

    double Flipper::GetAngle() {
        return m_Angle;
    }

    /* =============================================================== Controllers ===============================================================

       =========================================================================================================================================== */

    void RawFlipperController::ProcessCommand(Command& command) {
        m_Input = math::threshold(command.flipper, DEFAULT_INPUT_THRESHOLD);
        m_Output = m_Input * 0.25;
    }

    void RawFlipperController::Control() {
        auto flipper = m_Subsystem.lock();
        if (flipper->m_Robot->ShouldOutput()) {
            flipper->m_FlipperMaster.Set(m_Output);
        }
    }

    void RawFlipperController::Reset() {
        m_Input = 0.0;
        m_Output = 0.0;
    }

    void VelocityFlipperController::Reset() {
        m_WantedVelocity = 0.0;
        m_Input = 0.0;
    }

    void VelocityFlipperController::ProcessCommand(Command& command) {
        auto flipper = m_Subsystem.lock();
        m_Input = math::threshold(command.flipper, DEFAULT_INPUT_THRESHOLD);
        m_WantedVelocity = m_Input * flipper->m_MaxVelocity;
    }

    void VelocityFlipperController::Control() {
        auto flipper = m_Subsystem.lock();
        double encoderPosition = flipper->m_EncoderPosition;
        const bool inMiddle = encoderPosition > FLIPPER_SET_POINT_LOWER && encoderPosition < FLIPPER_SET_POINT_UPPER;
        bool wantingToGoOtherWay = false;
        if ((encoderPosition < FLIPPER_SET_POINT_LOWER && m_WantedVelocity > 0.01) ||
            (encoderPosition > FLIPPER_SET_POINT_UPPER && m_WantedVelocity < -0.01))
            wantingToGoOtherWay = true;
        if (inMiddle || wantingToGoOtherWay) {
            const double
                    angleFeedForward = std::cos(r2d(flipper->m_Angle)) * flipper->m_AngleFeedForward,
                    feedForward = angleFeedForward;
            if (flipper->m_Robot->ShouldOutput()) {
                auto error = flipper->m_FlipperController.SetReference(m_WantedVelocity, rev::ControlType::kSmartVelocity,
                                                                       FLIPPER_SMART_MOTION_PID_SLOT, feedForward);
                if (error == rev::CANError::kOK) {
                    Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format("Wanted Velocity: %f", m_WantedVelocity));
                } else {
                    Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("CAN Error: %d", error));
                }
            }
        } else {
            flipper->LogSample(lib::Logger::LogLevel::k_Debug, "Not doing anything");
            flipper->m_FlipperMaster.Set(0.0);
        }
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
                    angleFeedForward = std::cos(r2d(flipper->m_Angle)) * flipper->m_AngleFeedForward,
                    feedForward = angleFeedForward;
            if (flipper->m_Robot->ShouldOutput()) {
                auto error = flipper->m_FlipperController.SetReference(clampedSetPoint, rev::ControlType::kSmartMotion,
                                                                       FLIPPER_SMART_MOTION_PID_SLOT, feedForward);
                if (error == rev::CANError::kOK) {
                    Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format("Setting set point to: %d", clampedSetPoint));
                } else {
                    Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("CAN Error: %d", error));
                }
            }
        } else {
            flipper->LogSample(lib::Logger::LogLevel::k_Debug, "Not doing anything");
            flipper->m_FlipperMaster.Set(0.0);
        }
    }

    void SetPointFlipperController::Reset() {
        m_SetPoint = 0.0;
    }
}

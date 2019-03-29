#include <subsystem/flipper.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <lib/logger.hpp>

namespace garage {
//    template<>
//    class lib::SubsystemController<Flipper> {
//
//    };

    Flipper::Flipper(std::shared_ptr<Robot>& robot) : ControllableSubsystem(robot, "Flipper") {
        m_FlipperMaster.RestoreFactoryDefaults();
        m_FlipperMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_FlipperMaster.SetClosedLoopRampRate(FLIPPER_CLOSED_LOOP_RAMP);
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
        m_FlipperMaster.SetParameter(rev::CANSparkMax::ConfigParameter::kInputDeadband, 0.02);
        // TODO yay or nay with s-curve?
        m_FlipperController.SetSmartMotionAccelStrategy(rev::CANPIDController::AccelStrategy::kSCurve, FLIPPER_SMART_MOTION_PID_SLOT);
        m_FlipperMaster.EnableVoltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        m_ReverseLimitSwitch.EnableLimitSwitch(true);
        m_ForwardLimitSwitch.EnableLimitSwitch(true);
        m_FlipperMaster.Set(0.0);
    }

    void Flipper::OnPostInitialize() {
        auto flipper = WeakFromThis();
        AddController(m_RawController = std::make_shared<RawFlipperController>(flipper));
        AddController(m_SetPointController = std::make_shared<SetPointFlipperController>(flipper));
        AddController(m_VelocityController = std::make_shared<VelocityFlipperController>(flipper));
        SetUnlockedController(m_VelocityController);
//        SetupNetworkTableValues();
    }

    void Flipper::SetupNetworkTableValues() {
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
        m_FirstForwardLimitSwitchHit = true;
        m_FirstReverseLimitSwitchHit = true;
        m_IsForwardLimitSwitchDown = false;
        m_IsReverseLimitSwitchDown = false;
        m_LockServoOutput = LOCK_SERVO_LOWER;
    }

    bool Flipper::ShouldUnlock(Command& command) {
        return std::fabs(command.flipper) > DEFAULT_INPUT_THRESHOLD && m_LockServoOutput != LOCK_SERVO_UPPER;
    }

    void Flipper::HandleLimitSwitch(rev::CANDigitalInput& limitSwitch, bool& isLimitSwitchDown, bool& isFirstHit, double resetEncoderValue) {
        isLimitSwitchDown = limitSwitch.Get();
        if (isLimitSwitchDown) {
            if (isFirstHit) {
                auto error = m_Encoder.SetPosition(resetEncoderValue);
                if (error == rev::CANError::kOK) {
                    Log(lib::Logger::LogLevel::k_Info, "Limit switch hit and encoder reset");
                    isFirstHit = false;
                } else {
                    Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("CAN Error: %d", error));
                }
            }
        } else {
            isFirstHit = true;
        }
    }

    void Flipper::Update() {
        // Reverse limit switch
        HandleLimitSwitch(m_ReverseLimitSwitch, m_IsReverseLimitSwitchDown, m_FirstReverseLimitSwitchHit, FLIPPER_LOWER);
        HandleLimitSwitch(m_ForwardLimitSwitch, m_IsForwardLimitSwitchDown, m_FirstForwardLimitSwitchHit, FLIPPER_UPPER);
        // Encoder and angle
        m_EncoderPosition = m_Encoder.GetPosition();
        m_EncoderVelocity = m_Encoder.GetVelocity();
        m_Angle = RawSetPointToAngle(m_EncoderPosition);
        uint16_t faults = m_FlipperMaster.GetStickyFaults();
        faults &= ~(1 << 14);
        faults &= ~(1 << 15);
        if (faults) {
            Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("Sticky Fault Error: %d", faults));
            m_FlipperMaster.ClearFaults();
        }
        int cameraServoOutput;
        if (m_Angle < FLIPPER_CAMERA_STOW_THRESHOLD_ANGLE) {
            cameraServoOutput = CAMERA_SERVO_LOWER;
        } else if (m_Angle > FLIPPER_UPPER_ANGLE - FLIPPER_CAMERA_STOW_THRESHOLD_ANGLE) {
            cameraServoOutput = CAMERA_SERVO_UPPER;
        } else {
            cameraServoOutput = CAMERA_SERVO_MIDDLE;
        }
        m_CameraServoOutput = static_cast<uint16_t>(cameraServoOutput);
        m_CameraServo.SetRaw(m_CameraServoOutput);
//        m_LockServo.SetRaw(m_LockServoOutput);
        if (m_Controller) {
            m_Controller->Control();
        } else {
            LogSample(lib::Logger::LogLevel::k_Warning, "No controller detected");
        }
    }

    void Flipper::SpacedUpdate(Command& command) {
        const double appliedOutput = m_FlipperMaster.GetAppliedOutput(), current = m_FlipperMaster.GetOutputCurrent();
        m_NetworkTable->PutNumber("Angle", m_Angle);
        m_NetworkTable->PutNumber("Position", m_EncoderPosition);
        m_NetworkTable->PutNumber("Velocity", m_EncoderVelocity);
        m_NetworkTable->PutNumber("Output", appliedOutput);
        m_NetworkTable->PutNumber("Current", appliedOutput);
        Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format(
                "Output: %f, Current: %f, Angle: %f, Encoder Position: %f, Encoder Velocity: %f, Reverse Limit Switch: %s, Forward Limit Switch: %s",
                appliedOutput, current,
                m_Angle, m_EncoderPosition, m_EncoderVelocity,
                m_IsReverseLimitSwitchDown ? "true" : "false", m_IsForwardLimitSwitchDown ? "true" : "false"));
    }

    bool Flipper::IsWithinMotorOutputConditions(double wantedOutput, double forwardThreshold, double reverseThreshold) {
        double encoderPosition = m_EncoderPosition;
        const bool inMiddle = encoderPosition > FLIPPER_SET_POINT_LOWER && encoderPosition < FLIPPER_SET_POINT_UPPER;
        bool wantingToGoOtherWay = false;
        if ((encoderPosition < FLIPPER_SET_POINT_LOWER && wantedOutput > forwardThreshold) ||
            (encoderPosition > FLIPPER_SET_POINT_UPPER && wantedOutput < reverseThreshold)) {
            wantingToGoOtherWay = true;
        }
        return inMiddle || wantingToGoOtherWay;
    }

    void Flipper::SetRawOutput(double output) {
        SetController(m_RawController);
        m_RawController->SetOutput(output);
    }

    void Flipper::SetSetPoint(double setPoint) {
        SetController(m_SetPointController);
        m_SetPointController->SetSetPoint(setPoint);
    }

    void Flipper::SetAngle(double angle) {
        SetSetPoint(AngleToRawSetPoint(angle));
    }

    void Flipper::Stow() {
        SetAngle(FLIPPER_STOW_ANGLE);
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
        return math::withinRange(m_Angle, angle, FLIPPER_WITHIN_ANGLE);
    }

    double Flipper::GetAngle() {
        return m_Angle;
    }

    double Flipper::GetWantedAngle() {
        // TODO fix architecture
        auto controller = std::dynamic_pointer_cast<FlipperController>(m_Controller);
        return controller ? controller->GetWantedAngle() : GetAngle();
    }

    /* =============================================================== Controllers ===============================================================

       =========================================================================================================================================== */

    double FlipperController::GetWantedAngle() {
        return m_Subsystem.lock()->GetAngle();
    }

    void RawFlipperController::ProcessCommand(Command& command) {
        m_Output = command.flipper * FLIPPER_RAW_POWER;
    }

    void RawFlipperController::Control() {
        auto flipper = m_Subsystem.lock();
        Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format("Wanted Output: %f", m_Output));
        if (flipper->m_Robot->ShouldOutput()) {
            flipper->m_FlipperMaster.Set(m_Output);
        }
    }

    void RawFlipperController::SetOutput(double output) {
        m_Output = output;
    }

    void VelocityFlipperController::ProcessCommand(Command& command) {
        auto flipper = m_Subsystem.lock();
        m_WantedVelocity = command.flipper * flipper->m_MaxVelocity * FLIPPER_MANUAL_POWER;
    }

    void VelocityFlipperController::Control() {
        auto flipper = m_Subsystem.lock();
        if (flipper->IsWithinMotorOutputConditions(m_WantedVelocity, 0.0, 0.0)) {
            const double
                    angleFeedForward = std::cos(math::d2r(flipper->m_Angle + FLIPPER_COM_ANGLE_FF_OFFSET)) * flipper->m_AngleFeedForward,
                    feedForward = angleFeedForward;
            if (flipper->m_Robot->ShouldOutput()) {
                auto error = flipper->m_FlipperController.SetReference(m_WantedVelocity, rev::ControlType::kSmartVelocity,
                                                                       FLIPPER_SMART_MOTION_PID_SLOT, feedForward * DEFAULT_VOLTAGE_COMPENSATION);
                if (error == rev::CANError::kOK) {
                    flipper->LogSample(lib::Logger::LogLevel::k_Debug,
                                       lib::Logger::Format("Wanted Velocity: %f, Output Feed Forward: %f, Feed Forward: %f",
                                                           m_WantedVelocity, angleFeedForward, flipper->m_AngleFeedForward));
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
        m_SetPoint = math::clamp(m_SetPoint + command.flipper, FLIPPER_LOWER, FLIPPER_UPPER);
    }

    void SetPointFlipperController::Control() {
        auto flipper = m_Subsystem.lock();
        if (flipper->IsWithinMotorOutputConditions(m_SetPoint, FLIPPER_SET_POINT_LOWER, FLIPPER_SET_POINT_UPPER)) {
            const double
                    clampedSetPoint = math::clamp(m_SetPoint, FLIPPER_SET_POINT_LOWER, FLIPPER_SET_POINT_UPPER),
                    angleFeedForward = std::cos(math::d2r(flipper->m_Angle + FLIPPER_COM_ANGLE_FF_OFFSET)) * flipper->m_AngleFeedForward,
                    feedForward = angleFeedForward;
            if (flipper->m_Robot->ShouldOutput()) {
                auto error = flipper->m_FlipperController.SetReference(m_SetPoint, rev::ControlType::kSmartMotion,
                                                                       FLIPPER_SMART_MOTION_PID_SLOT, feedForward * DEFAULT_VOLTAGE_COMPENSATION);
                if (error == rev::CANError::kOK) {
                    Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format("Wanted set point: %f", m_SetPoint));
                } else {
                    Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("CAN Error: %d", error));
                }
            }
        } else {
            flipper->LogSample(lib::Logger::LogLevel::k_Debug, lib::Logger::Format("Not doing anything, wanted set point: %f", m_SetPoint));
            flipper->m_FlipperMaster.Set(0.0);
        }
    }
}

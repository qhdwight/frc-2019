#include <subsystem/flipper.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <lib/logger.hpp>

namespace garage {
    Flipper::Flipper(std::shared_ptr<Robot>& robot) : lib::ControllableSubsystem<Flipper>(robot, "Flipper") {
        m_FlipperMaster.RestoreFactoryDefaults();
        m_FlipperMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_FlipperMaster.SetOpenLoopRampRate(0.2);
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
        m_FlipperMaster.EnableVoltageCompensation(12.0);
        m_LimitSwitch.EnableLimitSwitch(false);
        auto flipper = std::dynamic_pointer_cast<Flipper>(shared_from_this());
        m_RawController = std::make_shared<RawFlipperController>(flipper);
        m_SetPointController = std::make_shared<SetPointFlipperController>(flipper);
        SetController(m_RawController);
    }

    void Flipper::TeleopInit() {

    }

    void Flipper::UpdateUnlocked(Command& command) {
        if (m_Controller) {
            m_Controller->ProcessCommand(command);
        } else {
            LogSample(lib::Logger::LogLevel::k_Warning, "No controller detected");
        }
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
        if (!m_IsLimitSwitchDown)
            m_FirstLimitSwitchHit = true;
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
        m_FlipperMaster.Set(output);
    }

    void Flipper::SetSetPoint(double setPoint) {
        SetController(m_SetPointController);
        auto error = m_FlipperController.SetReference(setPoint, rev::ControlType::kSmartMotion);
        if (error == rev::CANError::kOK) {
            Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("Setting set point to: %d", setPoint));
        } else {
            Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("CAN Error: %d", error));
        }
    }

    bool Flipper::SetController(std::shared_ptr<FlipperController> controller) {
        bool different = controller != m_Controller;
        if (different)
            m_Controller = controller;
        return different;
    }

    void RawFlipperController::ProcessCommand(Command& command) {
        m_Input = math::threshold(command.flipper, DEFAULT_INPUT_THRESHOLD);
    }

    void RawFlipperController::Control() {

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
            const double clampedSetPoint = math::clamp(m_SetPoint, FLIPPER_SET_POINT_LOWER, FLIPPER_SET_POINT_UPPER);
            flipper->SetSetPoint(clampedSetPoint);
        } else {
            flipper->LogSample(lib::Logger::LogLevel::k_Info, "Not doing anything");
            flipper->m_FlipperMaster.Set(0.0);
        }
    }
}

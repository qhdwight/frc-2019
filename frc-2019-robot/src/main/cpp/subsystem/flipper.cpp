#include <subsystem/flipper.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <lib/logger.hpp>

namespace garage {
    double kP = 4e-5, kI = 1e-7, kD = 5e-4, kIz = 3.0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;
    double kMaxVel = 4400, kMinVel = 0, kMaxAcc = 2300, kAllErr = 0.1;

    Flipper::Flipper(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Flipper") {
        m_Flipper.RestoreFactoryDefaults();
        m_Flipper.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_Flipper.SetOpenLoopRampRate(0.2);
        m_Flipper.SetClosedLoopRampRate(0.2);
        m_FlipperController.SetP(kP);
        m_FlipperController.SetI(kI);
        m_FlipperController.SetD(kD);
        m_FlipperController.SetIZone(kIz);
        m_FlipperController.SetFF(kFF);
        m_FlipperController.SetOutputRange(kMinOutput, kMaxOutput);
        m_FlipperController.SetSmartMotionMaxVelocity(kMaxVel);
        m_FlipperController.SetSmartMotionMinOutputVelocity(kMinVel);
        m_FlipperController.SetSmartMotionMaxAccel(kMaxAcc);
        m_FlipperController.SetSmartMotionAllowedClosedLoopError(kAllErr);
        m_LimitSwitch.EnableLimitSwitch(false);
//        frc::SmartDashboard::PutNumber("P Gain", kP);
//        frc::SmartDashboard::PutNumber("I Gain", kI);
//        frc::SmartDashboard::PutNumber("D Gain", kD);
//        frc::SmartDashboard::PutNumber("I Zone", kIz);
//        frc::SmartDashboard::PutNumber("Feed Forward", kFF);
//        frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
//        frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
//        frc::SmartDashboard::PutNumber("Max Velocity", kMaxVel);
//        frc::SmartDashboard::PutNumber("Min Velocity", kMinVel);
//        frc::SmartDashboard::PutNumber("Max Acceleration", kMaxAcc);
//        frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", kAllErr);
//        frc::SmartDashboard::PutNumber("Set Point", 0.0);
    }

    void Flipper::TeleopInit() {

    }

    void Flipper::ExecuteCommand(Command& command) {
//        const double output = command.flipper * 0.2;
//        m_FlipperController.SetReference(0.0, rev::ControlType::kSmartMotion);
//        m_Robot->GetNetworkTable()->PutNumber("Flipper/Flipper Amperage", m_Flipper.GetOutputCurrent());
//        m_Robot->GetNetworkTable()->PutNumber("Flipper/Flipper Output", output);
//        double p = frc::SmartDashboard::GetNumber("P Gain", 0);
//        double i = frc::SmartDashboard::GetNumber("I Gain", 0);
//        double d = frc::SmartDashboard::GetNumber("D Gain", 0);
//        double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
//        double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
//        double max = frc::SmartDashboard::GetNumber("Max Output", 0);
//        double min = frc::SmartDashboard::GetNumber("Min Output", 0);
//        double maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
//        double minV = frc::SmartDashboard::GetNumber("Min Velocity", 0);
//        double maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
//        double allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);
//        if((p != kP))   { m_FlipperController.SetP(p); kP = p; }
//        if((i != kI))   { m_FlipperController.SetI(i); kI = i; }
//        if((d != kD))   { m_FlipperController.SetD(d); kD = d; }
//        if((iz != kIz)) { m_FlipperController.SetIZone(iz); kIz = iz; }
//        if((ff != kFF)) { m_FlipperController.SetFF(ff); kFF = ff; }
//        if((max != kMaxOutput) || (min != kMinOutput)) { m_FlipperController.SetOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }
//        if((maxV != kMaxVel)) { m_FlipperController.SetSmartMotionMaxVelocity(maxV); kMaxVel = maxV; }
//        if((minV != kMinVel)) { m_FlipperController.SetSmartMotionMaxVelocity(minV); kMinVel = minV; }
//        if((maxA != kMaxAcc)) { m_FlipperController.SetSmartMotionMaxAccel(maxA); kMaxAcc = maxA; }
//        if((allE != kAllErr)) { m_FlipperController.SetSmartMotionAllowedClosedLoopError(allE); kAllErr = allE; }
//        frc::SmartDashboard::PutNumber("Encoder", m_Encoder.GetPosition());
//        frc::SmartDashboard::PutNumber("Current", m_Flipper.GetOutputCurrent());
//        frc::SmartDashboard::PutNumber("Output", m_Flipper.GetAppliedOutput());

        const bool isLimitSwitchClosed = m_LimitSwitch.Get();
        if (isLimitSwitchClosed && m_FirstLimitSwitchHit) {
            auto error = m_Encoder.SetPosition(0.0);
            if (error == rev::CANError::kOK) {
                Log(lib::LogLevel::kInfo, "Limit switch hit and encoder reset");
                m_FirstLimitSwitchHit = false;
            } else {
                Log(lib::LogLevel::kError, "CAN Error: " + std::to_string(static_cast<int>(error)));
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
                    Log(lib::LogLevel::kInfo, "Setting set point to: " + std::to_string(setPoint));
                    m_LastSetPoint = setPoint;
                } else {
                    Log(lib::LogLevel::kError, "CAN Error: " + std::to_string(static_cast<int>(error)));
                }
            }
        } else {
            LogSample(lib::LogLevel::kInfo, "Not doing anything");
            m_Flipper.Set(0.0);
        }
        if (faults != 0)
            LogSample(lib::LogLevel::kError, "Stick Fault Error: " + std::to_string(faults));
        LogSample(lib::LogLevel::kInfo, "Wanted Position: " + std::to_string(setPoint) + ", Output: " + std::to_string(m_Flipper.GetAppliedOutput()) + ", Limit Switch: " + std::to_string(isLimitSwitchClosed) + ", Encoder position: " + std::to_string(encoderPosition) + ", Encoder Velocity: " + std::to_string(m_Encoder.GetVelocity()) + ", Faults: " + std::to_string(faults));

//        const double output = math::threshold(m_LastCommand.ballIntake, 0.05) * 0.35;
//        m_Flipper.Set(output);
    }
}
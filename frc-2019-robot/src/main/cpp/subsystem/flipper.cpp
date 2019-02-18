#include <subsystem/flipper.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <lib/logger.hpp>

namespace garage {
    double kP = 3e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;
    double kMaxVel = 1200, kMinVel = 0, kMaxAcc = 800, kAllErr = 0;


    Flipper::Flipper(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Flipper") {
        m_Flipper.RestoreFactoryDefaults();
        m_Flipper.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
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
        if (isLimitSwitchClosed) {
            auto error = m_Encoder.SetPosition(0.0);
            if (error != rev::CANError::kOK)
                Log(lib::LogLevel::kError, "CAN Error: " + std::to_string(static_cast<int>(error)));
        }
        const auto encoderPosition = m_Encoder.GetPosition();
        const auto faults = m_Flipper.GetStickyFaults();
        if (faults != 0)
            Log(lib::LogLevel::kError, "Stick Fault Error: " + std::to_string(faults));
        m_Flipper.ClearFaults();
//        const double output = math::threshold(m_LastCommand.ballIntake, 0.05) * 0.35;
//        m_Flipper.Set(output);
        LogSample(lib::LogLevel::kInfo, "Limit Switch: " + std::to_string(isLimitSwitchClosed) + ", Output: " + std::to_string(m_Flipper.GetAppliedOutput()) + ", Encoder position: " + std::to_string(encoderPosition) + ", Encoder Velocity: " + std::to_string(m_Encoder.GetVelocity()) + ", Faults: " + std::to_string(faults), 3);
        if (m_Encoder.GetPosition() > FLIPPER_LOWER || m_Encoder.GetPosition() < FLIPPER_UPPER)
            m_FlipperController.SetReference(20.0, rev::ControlType::kSmartMotion);
        else
            m_Flipper.Set(0.0);
    }
}
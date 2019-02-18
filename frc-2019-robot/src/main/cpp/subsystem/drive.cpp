#include <subsystem/drive.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <cmath>

namespace garage {
    Drive::Drive(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Drive") {
        m_LeftSlave.RestoreFactoryDefaults();
        m_RightSlave.RestoreFactoryDefaults();
        m_LeftMaster.RestoreFactoryDefaults();
        m_RightMaster.RestoreFactoryDefaults();
        m_RightMaster.SetInverted(true);
        m_LeftSlave.Follow(m_LeftMaster);
        m_RightSlave.Follow(m_RightMaster);
        m_LeftSlave.SetOpenLoopRampRate(0.25);
        m_RightSlave.SetOpenLoopRampRate(0.25);
    }

    void Drive::TeleopInit() {
    }

    void Drive::SpacedUpdate() {
        Log(lib::LogLevel::kInfo, "Fwd: " + std::to_string(m_LastCommand.driveForward) + ", Trn: "  + std::to_string(m_LastCommand.driveTurn));
    }

    double Drive::InputFromCommand(double commandInput) {
        return std::abs(commandInput) > JOYSTICK_THRESHOLD ? (commandInput - math::sign(commandInput) * JOYSTICK_THRESHOLD) : 0.0;
    }

    void Drive::ExecuteCommand(Command& command) {
        if (!m_IsLocked) {
            const double forwardInput = InputFromCommand(command.driveForward), turnInput = InputFromCommand(command.driveTurn);
            m_LeftOutput = forwardInput + turnInput * (1 - math::abs(forwardInput) * 0.5) * 0.25;
            m_RightOutput = forwardInput - turnInput * (1 - math::abs(forwardInput) * 0.5) * 0.25;
        }
        LogSample(lib::LogLevel::kInfo, std::to_string(m_IsLocked) + ", " + std::to_string(m_LeftOutput) + ", " + std::to_string(m_RightOutput));
        m_LeftMaster.Set(m_LeftOutput);
        m_RightMaster.Set(m_RightOutput);
        m_Robot->GetNetworkTable()->PutNumber("Drive/Gyro", m_Pigeon.GetFusedHeading());
        m_Robot->GetNetworkTable()->PutNumber("Drive/Left Output", m_LeftOutput);
        m_Robot->GetNetworkTable()->PutNumber("Drive/Right Output", m_RightOutput);
        m_Robot->GetNetworkTable()->PutNumber("Drive/Left Encoder", m_LeftEncoder.GetPosition());
        m_Robot->GetNetworkTable()->PutNumber("Drive/Left Amperage", m_LeftMaster.GetOutputCurrent());
        m_Robot->GetNetworkTable()->PutNumber("Drive/Right Encoder", m_RightEncoder.GetPosition());
        m_Robot->GetNetworkTable()->PutNumber("Drive/Right Amperage", m_RightMaster.GetOutputCurrent());
    }

    double Drive::GetHeading() {
        return m_Pigeon.GetFusedHeading();
    }

    void Drive::ResetHeadingAndEncoders() {
        m_Pigeon.SetFusedHeading(0.0);
        m_LeftEncoder.SetPosition(0.0);
        m_RightEncoder.SetPosition(0.0);
    }

    void Drive::SetDriveOutput(double left, double right) {
        m_LeftOutput = left;
        m_RightOutput = right;
    }
}

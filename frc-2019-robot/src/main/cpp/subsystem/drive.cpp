#include <subsystem/drive.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <cmath>

namespace garage {
    Drive::Drive(std::shared_ptr<Robot>& robot) : lib::Subsystem(robot, "Drive") {
        m_LeftSlave.RestoreFactoryDefaults();
        m_RightSlave.RestoreFactoryDefaults();
        m_LeftMaster.RestoreFactoryDefaults();
        m_RightMaster.RestoreFactoryDefaults();
        m_RightMaster.SetInverted(true);
        m_LeftSlave.Follow(m_LeftMaster);
        m_RightSlave.Follow(m_RightMaster);
        m_LeftSlave.SetOpenLoopRampRate(0.2);
        m_RightSlave.SetOpenLoopRampRate(0.2);
    }

    void Drive::OnReset() {
        ResetGyroAndEncoders();
    }

    double Drive::InputFromCommand(double commandInput) {
        const double absoluteCommand = std::abs(commandInput), sign = math::sign(commandInput);
        return absoluteCommand > DEFAULT_INPUT_THRESHOLD ? sign * std::abs(std::pow(commandInput, 2.0)) : 0.0;
    }

    void Drive::UpdateUnlocked(Command& command) {
        const double
                forwardInput = InputFromCommand(command.driveForward),
                turnInput = InputFromCommand(command.driveTurn),
                forwardInputFine = math::threshold(command.driveForwardFine, DEFAULT_INPUT_THRESHOLD),
                turnInputFine = math::threshold(command.driveTurnFine, DEFAULT_INPUT_THRESHOLD);
        if (std::abs(forwardInput) > DEFAULT_INPUT_THRESHOLD || std::abs(turnInput) > DEFAULT_INPUT_THRESHOLD) {
            m_LeftOutput = forwardInput + turnInput * (1 - math::absolute(forwardInput) * 0.5) * 0.25;
            m_RightOutput = forwardInput - turnInput * (1 - math::absolute(forwardInput) * 0.5) * 0.25;
        } else {
            m_LeftOutput = (forwardInputFine + turnInputFine) * 0.05;
            m_RightOutput = (forwardInputFine - turnInputFine) * 0.05;
        }
    }

    void Drive::SpacedUpdate(Command& command) {
        const double
            leftOutput = m_LeftMaster.GetAppliedOutput(),
            rightOutput = m_RightMaster.GetAppliedOutput(),
            leftCurrent = m_LeftMaster.GetOutputCurrent(),
            rightCurrent = m_RightMaster.GetOutputCurrent();
        m_NetworkTable->PutNumber("Gyro", m_Pigeon.GetFusedHeading());
        m_NetworkTable->PutNumber("Left Output", leftOutput);
        m_NetworkTable->PutNumber("Right Output", rightOutput);
        m_NetworkTable->PutNumber("Left Encoder", m_LeftEncoder.GetPosition());
        m_NetworkTable->PutNumber("Left Amperage", leftCurrent);
        m_NetworkTable->PutNumber("Right Encoder", m_RightEncoder.GetPosition());
        m_NetworkTable->PutNumber("Right Amperage", rightCurrent);
        LogSample(lib::Logger::LogLevel::k_Info, lib::Logger::Format(
                "Left Output: %f, Right Output: %f, Left Current: %f, Right Current: %f",
                leftOutput, rightOutput, leftCurrent, rightCurrent));
    }

    void Drive::Update() {
        if (Robot::ShouldOutput) {
            m_LeftMaster.Set(m_LeftOutput);
            m_RightMaster.Set(m_RightOutput);
        }
    }

    double Drive::GetHeading() {
        return m_Pigeon.GetFusedHeading();
    }

    void Drive::ResetGyroAndEncoders() {
        m_Pigeon.SetFusedHeading(0.0);
        m_LeftEncoder.SetPosition(0.0);
        m_RightEncoder.SetPosition(0.0);
    }

    void Drive::SetDriveOutput(double left, double right) {
        m_LeftOutput = left;
        m_RightOutput = right;
    }

    double Drive::GetTilt() {
        double angles[3];
        m_Pigeon.GetYawPitchRoll(angles);
        return angles[1];
    }
}

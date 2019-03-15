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
        m_LeftSlave.SetOpenLoopRampRate(DRIVE_RAMPING);
        m_RightSlave.SetOpenLoopRampRate(DRIVE_RAMPING);
        m_LeftMaster.EnableVoltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        m_RightMaster.EnableVoltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        StopMotors();
    }

    void Drive::Reset() {
        Subsystem::Reset();
        StopMotors();
    }

    void Drive::StopMotors() {
        m_LeftOutput = 0.0;
        m_RightOutput = 0.0;
        m_LeftMaster.Set(0.0);
        m_RightMaster.Set(0.0);
    }

    bool Drive::ShouldUnlock(Command& command) {
        return std::fabs(command.driveForward) > DEFAULT_INPUT_THRESHOLD ||
               std::fabs(command.driveTurn) > DEFAULT_INPUT_THRESHOLD;
    }

    double Drive::InputFromCommand(double commandInput) {
        const double absoluteCommand = std::fabs(commandInput), sign = math::sign(commandInput);
        return absoluteCommand > DEFAULT_INPUT_THRESHOLD ? sign * std::fabs(std::pow(commandInput, 2.0)) : 0.0;
    }

    void Drive::UpdateUnlocked(Command& command) {
        if (command.drivePrecisionEnabled) {
            const double
                    forwardInputFine = math::threshold(command.driveForward, DEFAULT_INPUT_THRESHOLD) - DEFAULT_INPUT_THRESHOLD,
                    turnInputFine = math::threshold(command.driveTurn, DEFAULT_INPUT_THRESHOLD) - DEFAULT_INPUT_THRESHOLD;
            m_LeftOutput = (forwardInputFine + turnInputFine) * DRIVE_PRECISION_POWER;
            m_RightOutput = (forwardInputFine - turnInputFine) * DRIVE_PRECISION_POWER;
        } else {
            const double
                    forwardInput = InputFromCommand(command.driveForward),
                    turnInput = InputFromCommand(command.driveTurn);
            m_LeftOutput = forwardInput + turnInput * (1 - std::fabs(forwardInput) * 0.5) * 0.25;
            m_RightOutput = forwardInput - turnInput * (1 - std::fabs(forwardInput) * 0.5) * 0.25;
        }
    }

    void Drive::SpacedUpdate(Command& command) {
        const double
                leftOutput = m_LeftMaster.GetAppliedOutput(),
                rightOutput = m_RightMaster.GetAppliedOutput(),
                leftCurrent = m_LeftMaster.GetOutputCurrent(),
                rightCurrent = m_RightMaster.GetOutputCurrent();
        const double heading = m_Pigeon.GetFusedHeading(), fixedHeading = math::fixAngle(heading);
        m_NetworkTable->PutNumber("Gyro", fixedHeading);
        m_NetworkTable->PutNumber("Left Output", leftOutput);
        m_NetworkTable->PutNumber("Right Output", rightOutput);
        m_NetworkTable->PutNumber("Left Encoder", m_LeftEncoderPosition);
        m_NetworkTable->PutNumber("Left Current", leftCurrent);
        m_NetworkTable->PutNumber("Right Encoder", m_RightEncoderPosition);
        m_NetworkTable->PutNumber("Right Current", rightCurrent);
        Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format(
                "Left Output: %f, Right Output: %f, Left Current: %f, Right Current: %f",
                leftOutput, rightOutput, leftCurrent, rightCurrent));
    }

    void Drive::Update() {
        m_RightEncoderPosition = m_RightEncoder.GetPosition();
        m_LeftEncoderPosition = m_LeftEncoder.GetPosition();
        if (m_Robot->ShouldOutput()) {
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
        Lock();
        m_LeftOutput = left;
        m_RightOutput = right;
    }

    double Drive::GetTilt() {
        double angles[3];
        m_Pigeon.GetYawPitchRoll(angles);
        return angles[1];
    }

    int Drive::GetDiscreteRightEncoderTicks() {
        return std::lround(m_RightEncoderPosition * 100.0);
    }

    int Drive::GetDiscreteLeftEncoderTicks() {
        return std::lround(m_LeftEncoderPosition * 100.0);
    }
}

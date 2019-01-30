#include <subsystem/drive.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <cmath>

namespace garage {
    Drive::Drive(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
        Initialize();
    }

    void Drive::Initialize() {
        m_LeftMaster.ConfigFactoryDefault();
        m_RightMaster.ConfigFactoryDefault();
        m_LeftSlave.ConfigFactoryDefault();
        m_RightSlave.ConfigFactoryDefault();
        m_LeftSlave.Follow(m_LeftMaster);
        m_RightSlave.Follow(m_RightMaster);
        m_PoseEstimator = std::make_shared<lib::PoseEstimator>(m_LeftMaster, m_RightMaster, m_Pigeon);
    }

    void Drive::TeleopInit() {
        m_PoseEstimator->Reset();
    }

    double Drive::InputFromCommand(double commandInput) {
        return std::abs(commandInput) > JOYSTICK_THRESHOLD ? (commandInput - math::sign(commandInput) * JOYSTICK_THRESHOLD) : 0.0;
    }

    void Drive::ExecuteCommand(Command& command) {
        if (command.button)
            m_PoseEstimator->Reset();
        const double forwardInput = InputFromCommand(command.driveForward), turnInput = InputFromCommand(command.driveTurn),
                leftOutput = forwardInput + turnInput * (1 - std::abs(forwardInput) * 0.5),
                rightOutput = forwardInput - turnInput * (1 - std::abs(forwardInput) * 0.5);
        m_LeftMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, leftOutput);
        m_RightMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightOutput);
        auto pose = m_PoseEstimator->Update();
        auto gyroEntry = m_Robot->GetNetworkTable()->GetEntry("Gyro");
        auto leftEncoderEntry = m_Robot->GetNetworkTable()->GetEntry("Left Encoder");
        auto rightEncoderEntry = m_Robot->GetNetworkTable()->GetEntry("Right Encoder");
        auto poseEntry = m_Robot->GetNetworkTable()->GetEntry("Pose");
        gyroEntry.SetDouble(m_Pigeon.GetFusedHeading());
        leftEncoderEntry.SetDouble(m_LeftMaster.GetSelectedSensorPosition());
        rightEncoderEntry.SetDouble(m_RightMaster.GetSelectedSensorPosition());
        poseEntry.SetDoubleArray(wpi::ArrayRef<double>(pose.position.data()));
        Subsystem::ExecuteCommand(command);
    }
}

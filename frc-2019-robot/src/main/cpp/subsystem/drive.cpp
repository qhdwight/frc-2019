#include <subsystem/drive.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <cmath>

namespace garage {
    Drive::Drive(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
        m_LeftSlave.RestoreFactoryDefaults();
        m_RightSlave.RestoreFactoryDefaults();
        m_LeftMaster.RestoreFactoryDefaults();
        m_RightMaster.RestoreFactoryDefaults();
        m_RightMaster.SetInverted(true);
        m_LeftSlave.Follow(m_LeftMaster);
        m_RightSlave.Follow(m_RightMaster);
        m_LeftSlave.SetOpenLoopRampRate(0.25);
        m_RightSlave.SetOpenLoopRampRate(0.25);
//        m_PoseEstimator = std::make_shared<lib::PoseEstimator>(m_LeftEncoder, m_RightEncoder, m_Pigeon);
    }

    void Drive::TeleopInit() {
//        m_PoseEstimator->Reset();
    }

    double Drive::InputFromCommand(double commandInput) {
        return std::abs(commandInput) > JOYSTICK_THRESHOLD ? (commandInput - math::sign(commandInput) * JOYSTICK_THRESHOLD) : 0.0;
    }

    void Drive::ExecuteCommand(Command& command) {
//        if (command.button)
//            m_PoseEstimator->Reset();
        const double forwardInput = InputFromCommand(command.driveForward), turnInput = InputFromCommand(command.driveTurn),
                leftOutput = forwardInput + turnInput * (1 - std::abs(forwardInput) * 0.5) * 0.25,
                rightOutput = forwardInput - turnInput * (1 - std::abs(forwardInput) * 0.5) * 0.25;
        m_LeftMaster.Set(leftOutput);
        m_RightMaster.Set(rightOutput);
//        auto pose = m_PoseEstimator->Update();
        m_Robot->GetNetworkTable()->PutNumber("Drive/Gyro", m_Pigeon.GetFusedHeading());
        m_Robot->GetNetworkTable()->PutNumber("Drive/Left Output", leftOutput);
        m_Robot->GetNetworkTable()->PutNumber("Drive/Right Output", rightOutput);
        m_Robot->GetNetworkTable()->PutNumber("Drive/Left Encoder", m_LeftEncoder.GetPosition());
        m_Robot->GetNetworkTable()->PutNumber("Drive/Left Amperage", m_LeftMaster.GetOutputCurrent());
        m_Robot->GetNetworkTable()->PutNumber("Drive/Right Encoder", m_RightEncoder.GetPosition());
        m_Robot->GetNetworkTable()->PutNumber("Drive/Right Amperage", m_RightMaster.GetOutputCurrent());
//        m_Robot->GetNetworkTable()->PutNumberArray("Drive/Pose", wpi::ArrayRef<double>(pose.position.data()));
        Subsystem::ExecuteCommand(command);
    }
}

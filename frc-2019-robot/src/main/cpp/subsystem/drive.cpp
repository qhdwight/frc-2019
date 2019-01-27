#include <subsystem/drive.hpp>

#include <robot.hpp>

#include <cmath>

namespace garage {
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

    }

    void Drive::ExecuteCommand(Command& command) {
        if (command.button) {
            m_PoseEstimator->Reset();
        }
        const double
                forward = std::abs(command.forward) > JOYSTICK_THRESHOLD ? std::pow(command.forward, 1) : 0.0,
                turn = std::abs(command.turn) > JOYSTICK_THRESHOLD ? std::pow(command.turn, 1) : 0.0;
        m_LeftMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, forward + turn);
        m_RightMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, turn - forward);
        auto pose = m_PoseEstimator->Update();
        auto gyroEntry = m_Robot->GetNetworkTable()->GetEntry("Gyro");
        auto poseEntry = m_Robot->GetNetworkTable()->GetEntry("Pose");
        gyroEntry.SetDouble(m_Pigeon.GetFusedHeading());
        poseEntry.SetDoubleArray(wpi::ArrayRef<double>(pose.position.data()));
    }
}

#include <subsystem/drive.hpp>

#include <cmath>

namespace garage {
    void Drive::Initialize() {
        m_LeftMaster.ConfigFactoryDefault();
        m_RightMaster.ConfigFactoryDefault();
        m_LeftSlave.ConfigFactoryDefault();
        m_RightSlave.ConfigFactoryDefault();
        m_LeftSlave.Follow(m_LeftMaster);
        m_RightSlave.Follow(m_RightMaster);
        m_Pigeon.SetFusedHeading(0);
    }

    void Drive::ExecuteCommand(Command& command) {
        const double
                forward = std::abs(command.forward) > JOYSTICK_THRESHOLD ? std::pow(command.forward, 4) : 0.0,
                turn = std::abs(command.turn) > JOYSTICK_THRESHOLD ? std::pow(command.turn, 4) : 0.0;
        m_LeftMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, forward + turn);
        m_RightMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, forward - turn);

    }
}

#include <subsystem/drive.hpp>

namespace garage {
    void Drive::ExecuteCommand(Command& command) {
        const double
                forward = std::abs(command.forward) > JOYSTICK_THRESHOLD ? std::pow(command.forward, 3) : 0.0,
                turn = std::abs(command.turn) > JOYSTICK_THRESHOLD ? std::pow(command.turn, 3) : 0.0;
        m_LeftSRX.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, forward + turn);
        m_RightSRX.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, forward - turn);
    }

    Drive::Drive() : Subsystem() {
        m_LeftSPX.Follow(m_LeftSRX);
        m_RightSPX.Follow(m_RightSRX);
        m_LeftSPX.SetInverted(true);
    }
}

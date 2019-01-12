#include "robot.hpp"

namespace robot {
    void Robot::RobotInit() {}

    void Robot::DisabledInit() {}

    void Robot::AutonomousInit() {}

    void Robot::AutonomousPeriodic() {}

    void Robot::TeleopInit() {
        m_LeftSPX.Follow(m_LeftSRX);
        m_RightSPX.Follow(m_RightSRX);
        m_LeftSPX.SetInverted(true);
    }

    void Robot::TeleopPeriodic() {
        double forward = -m_Stick.GetY(), turn = m_Stick.GetX();
        m_LeftSRX.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, forward + turn * 2);
        m_RightSRX.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, forward - turn * 2);
    }
}

#ifndef RUNNING_FRC_TESTS

int main() {
    return frc::StartRobot<robot::Robot>();
}

#endif

#include <algorithm>

#include <robot.hpp>

#include <cameraserver/CameraServer.h>

namespace garage {
    void Robot::RobotInit() {
        m_NetworkTableInstace = nt::NetworkTableInstance::GetDefault();
        m_NetworkTable = m_NetworkTableInstace.GetTable("Garage Robotics");
        m_Subsystems.push_back(m_Drive);
        m_Subsystems.push_back(m_BallIntake);
    }

    void Robot::RobotPeriodic() {}

    void Robot::DisabledInit() {}

    void Robot::DisabledPeriodic() {}

    void Robot::AutonomousInit() {}

    void Robot::AutonomousPeriodic() {}

    void Robot::TeleopInit() {}

    void Robot::TeleopPeriodic() {
        Command command = GetCommand();
        for (const auto& subsystem : m_Subsystems)
            subsystem->ExecuteCommand(command);
    }

    Command Robot::GetCommand() {
        const double forward = -m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand),
                turn = m_Controller.GetX(frc::GenericHID::JoystickHand::kRightHand);
        const double intake = m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) -
                              m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
        return {forward, turn, intake, false};
        // return {-m_Stick.GetY(), m_Stick.GetX(), m_Stick.GetTrigger()};
    }

    std::shared_ptr<NetworkTable> Robot::GetNetworkTable() const {
        return m_NetworkTable;
    }
}

#ifndef RUNNING_FRC_TESTS

int main() {
    return frc::StartRobot<garage::Robot>();
}

#endif

#include <robot.hpp>

#include <algorithm>

#include <cameraserver/CameraServer.h>

namespace garage {
    void Robot::RobotInit() {
        auto robot = std::make_shared<Robot>();
        robot.reset(this);
        m_NetworkTableInstance = nt::NetworkTableInstance::GetDefault();
        m_NetworkTable = m_NetworkTableInstance.GetTable("Garage Robotics");
        m_Drive = std::make_shared<Drive>(robot);
//        m_Flipper = std::make_shared<Flipper>(robot);
        m_Elevator = std::make_shared<Elevator>(robot);
//        m_BallIntake = std::make_shared<BallIntake>(robot);
//        m_HatchIntake = std::make_shared<HatchIntake>(robot);
        m_Subsystems.push_back(m_Drive);
//        m_Subsystems.push_back(m_Flipper);
        m_Subsystems.push_back(m_Elevator);
//        m_Subsystems.push_back(m_BallIntake);
//        m_Subsystems.push_back(m_HatchIntake);
    }

    void Robot::RobotPeriodic() {}

    void Robot::DisabledInit() {}

    void Robot::DisabledPeriodic() {}

    void Robot::AutonomousInit() {}

    void Robot::AutonomousPeriodic() {}

    void Robot::TeleopInit() {
        for (const auto &subsystem : m_Subsystems)
            subsystem->TeleopInit();
    }

    void Robot::TeleopPeriodic() {
        Command command = GetCommand();
        for (const auto &subsystem : m_Subsystems)
            subsystem->ExecuteCommand(command);
    }

    Command Robot::GetCommand() {
        const double forward = m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand) * 1,
                turn = m_Controller.GetX(frc::GenericHID::JoystickHand::kRightHand) * 0.5;
        const double flipper = m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) -
                               m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
        const bool button = m_Controller.GetAButtonPressed();
        return {forward, turn, flipper, button};
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

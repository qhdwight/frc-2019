#include <robot.hpp>

#include <cameraserver/CameraServer.h>

namespace garage {
    void Robot::RobotInit() {
        // Robot is a stack object but I do not give a damn
        auto robot = std::shared_ptr<Robot>(this, [](auto robot) {});
        m_NetworkTableInstance = nt::NetworkTableInstance::GetDefault();
        m_NetworkTable = m_NetworkTableInstance.GetTable("Garage Robotics");
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_Flipper = std::make_shared<Flipper>(robot)));
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_Elevator = std::make_shared<Elevator>(robot)));
        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_Drive = std::make_shared<Drive>(robot)));
    }

    void Robot::AddSubsystem(std::shared_ptr<lib::Subsystem> subsystem) {
        m_Subsystems.push_back(subsystem);
    }

    void Robot::RobotPeriodic() {}

    void Robot::DisabledInit() {}

    void Robot::DisabledPeriodic() {}

    void Robot::AutonomousInit() {}

    void Robot::AutonomousPeriodic() {}

    void Robot::TeleopInit() {
        for (const auto& subsystem : m_Subsystems)
            subsystem->TeleopInit();
    }

    void Robot::TeleopPeriodic() {
        UpdateCommand();
        ExecuteCommand();
    }

    void Robot::UpdateCommand() {
        m_Command.driveForward = m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.driveTurn = m_Controller.GetX(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.flipper = m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) -
                            m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
        m_Command.button = m_Controller.GetAButtonPressed();
        m_Command.hatchIntakeDown = m_Controller.GetBButtonPressed();
        m_Command.elevatorPosition -= math::threshold(m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand) * 500.0, 50.0);
        m_Command.elevatorPosition = math::clamp(m_Command.elevatorPosition, 0.0, 120000.0);
        m_Command.test = -m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.routines.clear();
    }

    void Robot::ExecuteCommand() {
        for (const auto& subsystem : m_Subsystems)
            subsystem->ExecuteCommand(m_Command);
        m_RoutineManager->AddRoutinesFromCommand(m_Command);
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

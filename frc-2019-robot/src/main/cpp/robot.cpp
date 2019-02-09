#include <robot.hpp>

#include <cameraserver/CameraServer.h>

namespace garage {
    void Robot::RobotInit() {
        auto robot = std::make_shared<Robot>();
        robot.reset(this);
        m_NetworkTableInstance = nt::NetworkTableInstance::GetDefault();
        m_NetworkTable = m_NetworkTableInstance.GetTable("Garage Robotics");
        m_NetworkTable->PutString("Test Entry 1", "Test Value 1");
        m_NetworkTable->PutString("Test Entry 2", "Test Value 2");
//        AddSubsystem(std::make_shared<Drive>(robot));
//        AddSubsystem(std::make_shared<Flipper>(robot));
//        AddSubsystem(std::make_shared<Elevator>(robot));
//        AddSubsystem(std::make_shared<BallIntake>(robot));
//        AddSubsystem(std::make_shared<HatchIntake>(robot));
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
        m_Command.elevatorPosition = std::min(m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand), 0.0) * ELEVATOR_HIGHER_POSITION;
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

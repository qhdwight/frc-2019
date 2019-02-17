#include <robot.hpp>

#include <test/test_elevator_routine.hpp>

#include <lib/wait_routine.hpp>

namespace garage {
    void Robot::RobotInit() {
        // Robot is a stack object but I do not give a damn
        m_Pointer = std::shared_ptr<Robot>(this, [](auto robot) {});
        m_NetworkTableInstance = nt::NetworkTableInstance::GetDefault();
        m_NetworkTable = m_NetworkTableInstance.GetTable("Garage Robotics");
        m_RoutineManager = std::make_shared<lib::RoutineManager>(m_Pointer);
        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_Elevator = std::make_shared<Elevator>(m_Pointer)));
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_Drive = std::make_shared<Drive>(robot)));
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_Flipper = std::make_shared<Flipper>(robot)));
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_BallIntake = std::make_shared<BallIntake>(robot)));
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_HatchIntake = std::make_shared<HatchIntake>(robot)));
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_Outrigger = std::make_shared<Outrigger>(robot)));
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
        m_Command.driveForward = -m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.driveTurn = m_Controller.GetX(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.ballIntake = m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) -
                               m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
        m_Command.button = m_Controller.GetAButtonPressed();
        m_Command.hatchIntakeDown = m_Controller.GetBButtonPressed();
        m_Command.flipper = (m_Controller.GetBumper(frc::GenericHID::JoystickHand::kRightHand) ? 1.0 : 0.0) + (m_Controller.GetBumper(
                frc::GenericHID::JoystickHand::kLeftHand) ? -1.0 : 0.0);
        m_Command.elevatorPosition -= math::threshold(static_cast<int>(m_Controller.GetY(frc::GenericHID::JoystickHand::kLeftHand) * 5000.0), 50);
        m_Command.elevatorPosition = math::clamp(m_Command.elevatorPosition, ELEVATOR_MIN, ELEVATOR_MAX);
        m_Command.test = m_Controller.GetY(frc::GenericHID::JoystickHand::kLeftHand);
        m_Command.routines.clear();
        if (m_Controller.GetXButtonPressed()) {
            m_Command.routines.push_back(std::make_shared<test::TestElevatorRoutine>(m_Pointer, 100000.0));
            m_Command.routines.push_back(std::make_shared<lib::WaitRoutine>(m_Pointer, 0.2));
            m_Command.routines.push_back(std::make_shared<test::TestElevatorRoutine>(m_Pointer, 0.0));
        }
    }

    void Robot::ExecuteCommand() {
        for (const auto& subsystem : m_Subsystems)
            subsystem->ExecuteCommand(m_Command);
        m_RoutineManager->AddRoutinesFromCommand(m_Command);
    }

    std::shared_ptr<NetworkTable> Robot::GetNetworkTable() const {
        return m_NetworkTable;
    }

    std::shared_ptr<Elevator> Robot::GetElevator() {
        return m_Elevator;
    }
}

#ifndef RUNNING_FRC_TESTS

int main() {
    return frc::StartRobot<garage::Robot>();
}

#endif

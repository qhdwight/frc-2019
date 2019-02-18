#include <robot.hpp>

#include <test/test_elevator_routine.hpp>

#include <lib/logger.hpp>
#include <lib/wait_routine.hpp>
#include <lib/sequential_routine.hpp>

namespace garage {
    void Robot::RobotInit() {
        const auto defaultLogLevel = lib::LogLevel::kInfo;
        m_Logger = std::make_shared<lib::Logger>();
        m_Logger->SetLogLevel(defaultLogLevel);
        m_Logger->Log(lib::LogLevel::kInfo, "Robot initialized");
        m_Pointer = std::shared_ptr<Robot>(this, [](auto robot) {});
        m_NetworkTableInstance = nt::NetworkTableInstance::GetDefault();
        m_NetworkTable = m_NetworkTableInstance.GetTable("Garage Robotics");
        m_RoutineManager = std::make_shared<lib::RoutineManager>(m_Pointer);
        m_NetworkTable->PutNumber("Log Level", static_cast<double>(defaultLogLevel));
        m_NetworkTable->GetEntry("Log Level").AddListener([&](const nt::EntryNotification& notification) {
            auto logLevel = static_cast<lib::LogLevel>(std::round(notification.value->GetDouble()));
            m_Logger->SetLogLevel(logLevel);
            m_Logger->Log(lib::LogLevel::kInfo, "Updated log level to: " + std::to_string(logLevel));
        }, 0x10);
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_Elevator = std::make_shared<Elevator>(m_Pointer)));
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_Drive = std::make_shared<Drive>(m_Pointer)));
        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_Flipper = std::make_shared<Flipper>(m_Pointer)));
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_BallIntake = std::make_shared<BallIntake>(m_Pointer)));
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_HatchIntake = std::make_shared<HatchIntake>(m_Pointer)));
//        AddSubsystem(std::dynamic_pointer_cast<lib::Subsystem>(m_Outrigger = std::make_shared<Outrigger>(m_Pointer)));
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
//        auto r1 = std::dynamic_pointer_cast<lib::Routine>(std::make_shared<lib::WaitRoutine>(m_Pointer, "Wait One", 2.0));
//        auto r2 = std::dynamic_pointer_cast<lib::Routine>(std::make_shared<lib::WaitRoutine>(m_Pointer, "Wait Two", 4.0));
//        std::vector<std::shared_ptr<lib::Routine>> routines {r1, r2};
//        auto seq = std::dynamic_pointer_cast<lib::Routine>(std::make_shared<lib::SequentialRoutine>(m_Pointer, "All", std::move(routines)));
//        m_RoutineManager->AddRoutine(seq);
        m_Command = {};
        for (const auto& subsystem : m_Subsystems)
            subsystem->TeleopInit();
    }

    void Robot::TeleopPeriodic() {
        m_RoutineManager->Update();
        UpdateCommand();
        ExecuteCommand();
    }

    void Robot::UpdateCommand() {
        m_Command.driveForward = -m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.driveTurn = m_Controller.GetX(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.ballIntake = m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) -
                               m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
        m_Command.hatchIntakeDown = m_Controller.GetYButtonPressed();
        m_Command.flipper += ((m_Controller.GetBumper(frc::GenericHID::JoystickHand::kRightHand) ? 1.0 : 0.0) + (m_Controller.GetBumper(
                frc::GenericHID::JoystickHand::kLeftHand) ? -1.0 : 0.0)) * 0.25;
        m_Command.flipper = math::clamp(m_Command.flipper, 2.0, 36.0);
        m_Command.elevatorPosition -= math::threshold(static_cast<int>(m_Controller.GetY(frc::GenericHID::JoystickHand::kLeftHand) * 5000.0), 500);
        m_Command.elevatorPosition = math::clamp(m_Command.elevatorPosition, ELEVATOR_MIN, ELEVATOR_MAX);
        m_Command.test = m_Controller.GetY(frc::GenericHID::JoystickHand::kLeftHand);
        m_Command.routines.clear();
        if (m_Controller.GetAButtonPressed()) {
            m_Command.routines.push_back(std::make_shared<test::TestElevatorRoutine>(m_Pointer, "Elevator Up", 100000.0));
            m_Command.routines.push_back(std::make_shared<lib::WaitRoutine>(m_Pointer, "Elevator Wait", 0.2));
            m_Command.routines.push_back(std::make_shared<test::TestElevatorRoutine>(m_Pointer, "Elevator Down", 0.0));
        }
        if (m_Controller.GetBButtonPressed()) {
            if (m_RoutineManager)
                m_RoutineManager->TerminateAllRoutines();
        }
    }

    void Robot::ExecuteCommand() {
        for (const auto& subsystem : m_Subsystems)
            subsystem->Update(m_Command);
        if (m_RoutineManager)
            m_RoutineManager->AddRoutinesFromCommand(m_Command);
    }

    std::shared_ptr<NetworkTable> Robot::GetNetworkTable() const {
        return m_NetworkTable;
    }

    std::shared_ptr<Elevator> Robot::GetElevator() {
        return m_Elevator;
    }

    std::shared_ptr<Drive> Robot::GetDrive() {
        return m_Drive;
    }

    std::shared_ptr<lib::Logger> Robot::GetLogger() {
        return m_Logger;
    }
}

#ifndef RUNNING_FRC_TESTS

int main() {
    return frc::StartRobot<garage::Robot>();
}

#endif

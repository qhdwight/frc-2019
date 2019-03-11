#include <robot.hpp>

#include <lib/wait_routine.hpp>

#include <wpi/json.h>
#include <wpi/Path.h>
#include <wpi/FileSystem.h>
#include <wpi/raw_istream.h>

#include <frc/Filesystem.h>

namespace garage {
    void Robot::RobotInit() {
        lib::Logger::Log(lib::Logger::LogLevel::k_Info, "Start robot initialization");
        m_Period = std::chrono::milliseconds(std::lround(m_period * 1000.0));
        // Setup network tables
        m_NetworkTableInstance = nt::NetworkTableInstance::GetDefault();
        m_NetworkTable = m_NetworkTableInstance.GetTable("Garage Robotics");
        // Setup logging system
        const auto defaultLogLevel = lib::Logger::LogLevel::k_Info;
        lib::Logger::SetLogLevel(defaultLogLevel);
        m_NetworkTable->PutNumber("Log Level", static_cast<double>(defaultLogLevel));
        m_NetworkTable->GetEntry("Log Level").AddListener([&](const nt::EntryNotification& notification) {
            auto logLevel = static_cast<lib::Logger::LogLevel>(std::lround(notification.value->GetDouble()));
            lib::Logger::SetLogLevel(logLevel);
            lib::Logger::Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("Updated log level to: %d", logLevel));
        }, NT_NOTIFY_UPDATE);
        m_Pointer = std::shared_ptr<Robot>(this, [](auto robot) {});
        // Setup routine manager
        m_RoutineManager = std::make_shared<lib::RoutineManager>(m_Pointer);
        // Read the json configuration file on the rio
        wpi::SmallString<256> deployDirectory;
        frc::filesystem::GetDeployDirectory(deployDirectory);
        wpi::sys::path::append(deployDirectory, "settings.json");
        std::error_code errorCode;
        wpi::raw_fd_istream settingsFile(deployDirectory, errorCode);
        bool enableElevator = true, enableDrive = true, enableFlipper = true, enableBallIntake = true, enableHatchIntake = true, enableOutrigger = true;
        if (errorCode) {
            lib::Logger::Log(lib::Logger::LogLevel::k_Error,
                             lib::Logger::Format("Error reading robot settings file: %s", FMT_STR(errorCode.message())));
        } else {
            try {
                auto json = wpi::json::parse(settingsFile);
                lib::Logger::Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("Read settings: %s", FMT_STR(json.dump())));
                m_ShouldOutputMotors = json.at("should_output_motors").get<bool>();
                enableElevator = json.at("enable_elevator").get<bool>();
                enableDrive = json.at("enable_drive").get<bool>();
                enableFlipper = json.at("enable_flipper").get<bool>();
                enableBallIntake = json.at("enable_ball_intake").get<bool>();
                enableHatchIntake = json.at("enable_hatch_intake").get<bool>();
                enableOutrigger = json.at("enable_outrigger").get<bool>();
            } catch (wpi::detail::parse_error& error) {
                lib::Logger::Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("Error parsing robot settings: %s", error.what()));
            }
        }
        // Manage subsystems
        if (enableElevator) AddSubsystem(m_Elevator = std::make_shared<Elevator>(m_Pointer));
        if (enableDrive) AddSubsystem(m_Drive = std::make_shared<Drive>(m_Pointer));
        if (enableFlipper) AddSubsystem(m_Flipper = std::make_shared<Flipper>(m_Pointer));
        if (enableBallIntake) AddSubsystem(m_BallIntake = std::make_shared<BallIntake>(m_Pointer));
        if (enableHatchIntake) AddSubsystem(m_HatchIntake = std::make_shared<HatchIntake>(m_Pointer));
        if (enableOutrigger) AddSubsystem(m_Outrigger = std::make_shared<Outrigger>(m_Pointer));
        /* Create routines */
        // Render drive trajectory in initialization because it takes a couple of seconds
        m_DriveForwardRoutine = std::make_shared<test::TestDriveAutoRoutine>(m_Pointer, "Test Drive");
        m_DriveForwardRoutine->CalculatePath();
        m_LowerElevatorRoutine = std::make_shared<SetElevatorPositionRoutine>(m_Pointer, 0, "Lower Elevator");
        // Hatch routines
        m_BottomHatchRoutine = std::make_shared<SetElevatorPositionRoutine>(m_Pointer, 10000, "Bottom Hatch");
        m_MiddleHatchRoutine = std::make_shared<SetElevatorPositionRoutine>(m_Pointer, 50000, "Middle Hatch");
        m_TopHatchRoutine = std::make_shared<SetElevatorPositionRoutine>(m_Pointer, 100000, "Top Hatch");
        // Ball routines
        m_BottomHatchRoutine = std::make_shared<SetElevatorPositionRoutine>(m_Pointer, 10000, "Bottom Ball");
        m_MiddleHatchRoutine = std::make_shared<SetElevatorPositionRoutine>(m_Pointer, 50000, "Middle Ball");
        m_TopHatchRoutine = std::make_shared<SetElevatorPositionRoutine>(m_Pointer, 100000, "Top Ball");
        lib::Logger::Log(lib::Logger::LogLevel::k_Info, "End robot initialization");
    }

    void Robot::AddSubsystem(std::shared_ptr<lib::Subsystem> subsystem) {
        m_Subsystems.push_back(subsystem);
        subsystem->PostInitialize();
    }

    void Robot::RobotPeriodic() {}

    void Robot::DisabledInit() {}

    void Robot::DisabledPeriodic() {}

    void Robot::AutonomousInit() {
        Reset();
    }

    void Robot::AutonomousPeriodic() {
        MatchPeriodic();
    }

    void Robot::Reset() {
        m_Command = {};
        m_RoutineManager->Reset();
        for (const auto& subsystem : m_Subsystems)
            subsystem->Reset();
    }

    void Robot::TeleopInit() {
        Reset();
//        auto r1 = std::dynamic_pointer_cast<lib::Routine>(std::make_shared<lib::WaitRoutine>(m_Pointer, "Wait One", 2.0));
//        auto r2 = std::dynamic_pointer_cast<lib::Routine>(std::make_shared<lib::WaitRoutine>(m_Pointer, "Wait Two", 4.0));
//        std::vector<std::shared_ptr<lib::Routine>> routines{r1, r2};
//        auto seq = std::dynamic_pointer_cast<lib::Routine>(std::make_shared<lib::SequentialRoutine>(m_Pointer, "All", std::move(routines)));
//        m_RoutineManager->AddRoutine(seq);
    }

    void Robot::MatchPeriodic() {
        // See if we are taking too much time and not getting fifty updates a second
        auto now = std::chrono::system_clock::now();
        if (m_LastPeriodicTime) {
            auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_LastPeriodicTime.value());
            if (delta > m_Period * 1.05) {
                lib::Logger::Log(lib::Logger::LogLevel::k_Warning,
                                 lib::Logger::Format("Loop was more than 5% of expected, took %d milliseconds", delta));
            }
        }
        m_LastPeriodicTime = now;
        // Get our most up to date command from the controllers
        UpdateCommand();
        // Make sure our routine manager updates
        if (m_RoutineManager)
            m_RoutineManager->AddRoutinesFromCommand(m_Command);
        m_RoutineManager->Update();
        // Update each subsystem
        for (const auto& subsystem : m_Subsystems)
            subsystem->Periodic();
    }

    void Robot::TeleopPeriodic() {
        MatchPeriodic();
    }

    void Robot::UpdateCommand() {
        m_Command.driveForward = -m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.driveTurn = m_Controller.GetX(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.driveForwardFine = -m_Controller.GetY(frc::GenericHID::JoystickHand::kLeftHand);
        m_Command.driveTurnFine = m_Controller.GetX(frc::GenericHID::JoystickHand::kLeftHand);
//        m_Command.ballIntake = m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) -
//                               m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
//        m_Command.hatchIntakeDown = m_Controller.GetYButtonPressed();
//        m_Command.flipper = math::axis<double>(
//                m_Controller.GetBumper(frc::GenericHID::JoystickHand::kRightHand),
//                m_Controller.GetBumper(frc::GenericHID::JoystickHand::kLeftHand));
        const int pov = m_Controller.GetPOV();
        const bool
                elevatorHatch = pov == 90,
                elevatorDown = pov == 180,
                elevatorBall = pov == 270,
                elevatorSoftLand = pov == 0;
        m_Command.routines.clear();
//        if (m_Controller.GetAButtonPressed()) {
//            m_Command.routines.push_back(std::make_shared<test::SetElevatorPositionRoutine>(m_Pointer, "Elevator Up", 100000.0));
//            m_Command.routines.push_back(std::make_shared<lib::WaitRoutine>(m_Pointer, "Elevator Wait", 0.2));
//            m_Command.routines.push_back(std::make_shared<test::SetElevatorPositionRoutine>(m_Pointer, "Elevator Down", 0.0));
//        }
//        m_Command.elevatorSoftLand = m_Controller.GetBButtonPressed();
//        if (m_Controller.GetBButtonPressed()) {
//            if (m_RoutineManager)
//                m_RoutineManager->TerminateAllRoutines();
//        }
        if (m_Controller.GetAButtonPressed()) {
            m_RoutineManager->TerminateAllRoutines();
            m_Command.routines.push_back(m_DriveForwardRoutine);
        }
        if (elevatorDown) {
            m_RoutineManager->TerminateAllRoutines();
            m_Command.routines.push_back(m_LowerElevatorRoutine);
        }
        if (elevatorBall) {
            m_RoutineManager->TerminateAllRoutines();
            m_Command.routines.push_back(m_BottomBallRoutine);
        }
        if (elevatorHatch) {
            m_RoutineManager->TerminateAllRoutines();
            m_Command.routines.push_back(m_BottomHatchRoutine);
        }
        if (elevatorSoftLand) {
            m_RoutineManager->TerminateAllRoutines();
            m_Elevator->SoftLand();
        }
        m_Command.elevatorInput = -m_Controller.GetY(frc::GenericHID::kRightHand);
    }
}

#ifndef RUNNING_FRC_TESTS

int main() {
    return frc::StartRobot<garage::Robot>();
}

#endif

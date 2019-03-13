#include <robot.hpp>

#include <lib/wait_routine.hpp>
#include <lib/sequential_routine.hpp>

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
        ReadConfig();
        // Manage subsystems
        if (m_Config.enableElevator) AddSubsystem(m_Elevator = std::make_shared<Elevator>(m_Pointer));
        if (m_Config.enableDrive) AddSubsystem(m_Drive = std::make_shared<Drive>(m_Pointer));
        if (m_Config.enableFlipper) AddSubsystem(m_Flipper = std::make_shared<Flipper>(m_Pointer));
        if (m_Config.enableBallIntake) AddSubsystem(m_BallIntake = std::make_shared<BallIntake>(m_Pointer));
        if (m_Config.enableHatchIntake) AddSubsystem(m_HatchIntake = std::make_shared<HatchIntake>(m_Pointer));
        if (m_Config.enableOutrigger) AddSubsystem(m_Outrigger = std::make_shared<Outrigger>(m_Pointer));
        /* Create routines */
        // Render drive trajectory in initialization because it takes a couple of seconds
//        m_DriveForwardRoutine = std::make_shared<test::TestDriveAutoRoutine>(m_Pointer, "Test Drive");
//        m_DriveForwardRoutine->CalculatePath();
        auto testWaitRoutine = std::make_shared<lib::WaitRoutine>(m_Pointer, 500l);
        m_ResetRoutine = std::make_shared<ResetRoutine>(m_Pointer);
        // Hatch routines
        m_BottomHatchRoutine = std::make_shared<ElevatorAndFlipperRoutine>(m_Pointer, m_Config.bottomHatchHeight, 180.0, "Bottom Hatch");
        m_MiddleHatchRoutine = std::make_shared<ElevatorAndFlipperRoutine>(m_Pointer, m_Config.middleHatchHeight, 180.0, "Middle Hatch");
        m_TopHatchRoutine = std::make_shared<ElevatorAndFlipperRoutine>(m_Pointer, m_Config.topHatchHeight, 180.0, "Top Hatch");
        // Ball routines
        m_BottomBallRoutine = std::make_shared<BallPlacementRoutine>(m_Pointer, m_Config.bottomBallHeight, m_Config.bottomBallAngle, "Bottom Ball");
        m_MiddleBallRoutine = std::make_shared<BallPlacementRoutine>(m_Pointer, m_Config.middleBallHeight, m_Config.middleBallAngle, "Middle Ball");
        m_TopBallRoutine = std::make_shared<BallPlacementRoutine>(m_Pointer, m_Config.topBallHeight, m_Config.bottomBallAngle, "Top Ball");
        m_TestRoutine = std::make_shared<lib::SequentialRoutine>(m_Pointer, "Test Routine",
                                                                 lib::RoutineVector{testWaitRoutine, testWaitRoutine, testWaitRoutine});
        lib::Logger::Log(lib::Logger::LogLevel::k_Info, "End robot initialization");
    }

    void Robot::ReadConfig() {
        wpi::SmallString<PATH_LENGTH> deployDirectory;
        frc::filesystem::GetDeployDirectory(deployDirectory);
        wpi::sys::path::append(deployDirectory, "settings.json");
        std::error_code errorCode;
        wpi::raw_fd_istream settingsFile(deployDirectory, errorCode);
        if (errorCode) {
            lib::Logger::Log(lib::Logger::LogLevel::k_Error,
                             lib::Logger::Format("Error reading robot settings file: %s", FMT_STR(errorCode.message())));
        } else {
            try {
                auto json = wpi::json::parse(settingsFile);
                lib::Logger::Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("Read settings: %s", FMT_STR(json.dump())));
                m_Config.shouldOutput = json.at("should_output_motors").get<bool>();
                {
                    auto subsystemsJson = json.at("subsystems");
                    m_Config.enableElevator = subsystemsJson.at("enable_elevator").get<bool>();
                    m_Config.enableDrive = subsystemsJson.at("enable_drive").get<bool>();
                    m_Config.enableFlipper = subsystemsJson.at("enable_flipper").get<bool>();
                    m_Config.enableBallIntake = subsystemsJson.at("enable_ball_intake").get<bool>();
                    m_Config.enableHatchIntake = subsystemsJson.at("enable_hatch_intake").get<bool>();
                    m_Config.enableOutrigger = subsystemsJson.at("enable_outrigger").get<bool>();
                }
                {
                    auto routinesJson = json.at("routines");
                    {
                        auto elevatorRoutines = routinesJson.at("elevator");
                        {
                            auto ballRoutines = elevatorRoutines.at("ball");
                            // Set Points
                            m_Config.bottomBallHeight = ballRoutines.at("bottom_set_point").get<int>();
                            m_Config.middleBallHeight = ballRoutines.at("middle_set_point").get<int>();
                            m_Config.topBallHeight = ballRoutines.at("top_set_point").get<int>();
                            // Angles
                            m_Config.bottomBallAngle = ballRoutines.at("bottom_angle").get<double>();
                            m_Config.middleBallAngle = ballRoutines.at("middle_angle").get<double>();
                            m_Config.topBallAngle = ballRoutines.at("top_angle").get<double>();
                        }
                        {
                            auto hatchRoutines = elevatorRoutines.at("hatch");
                            m_Config.bottomHatchHeight = hatchRoutines.at("bottom").get<int>();
                            m_Config.middleHatchHeight = hatchRoutines.at("middle").get<int>();
                            m_Config.topHatchHeight = hatchRoutines.at("top").get<int>();
                        }
                    }
                }
            } catch (wpi::detail::parse_error& error) {
                lib::Logger::Log(lib::Logger::LogLevel::k_Error, lib::Logger::Format("Error parsing robot settings: %s", error.what()));
            }
        }
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
        ControllablePeriodic();
    }

    void Robot::Reset() {
        m_LastPeriodicTime.reset();
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

    void Robot::ControllablePeriodic() {
        // See if we are taking too much time and not getting fifty updates a second
        auto now = std::chrono::system_clock::now();
        if (m_LastPeriodicTime) {
            auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_LastPeriodicTime.value());
            if (delta > m_Period * 1.05) {
                lib::Logger::Log(lib::Logger::LogLevel::k_Warning,
                                 lib::Logger::Format("Loop was more than five percent of expected, took %d milliseconds", delta));
            }
        }
        m_LastPeriodicTime = now;
        // Get our most up to date command from the controllers
        UpdateCommand();
        // Make sure our routine manager updates
        m_RoutineManager->AddRoutinesFromCommand(m_Command);
        m_RoutineManager->Update();
        // Update each subsystem
        for (const auto& subsystem : m_Subsystems)
            subsystem->Periodic();
    }

    void Robot::TeleopPeriodic() {
        ControllablePeriodic();
    }

    void Robot::UpdateCommand() {
        if (m_Controller.GetStickButtonPressed(frc::GenericHID::kRightHand)) {
            m_Command.drivePrescisionEnabled = !m_Command.drivePrescisionEnabled;
        }
        if (m_Controller.GetStickButtonPressed(frc::GenericHID::kLeftHand)) {
            m_Command.elevatorOpenLoopEnabled = !m_Command.elevatorOpenLoopEnabled;
        }
        m_Command.driveForward = -m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.driveTurn = m_Controller.GetX(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.flipper = m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) -
                            m_Controller.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
        m_Command.ballIntake = math::axis<double>(
                m_Controller.GetBumper(frc::GenericHID::JoystickHand::kRightHand),
                m_Controller.GetBumper(frc::GenericHID::JoystickHand::kLeftHand));
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
            if (elevatorBall) {
                m_Command.routines.push_back(m_BottomBallRoutine);
            } else if (elevatorHatch) {
                m_Command.routines.push_back(m_BottomHatchRoutine);
            }
        }
        if (m_Controller.GetBButtonPressed()) {
            if (elevatorBall) {
                m_Command.routines.push_back(m_MiddleBallRoutine);
            } else if (elevatorHatch) {
                m_Command.routines.push_back(m_MiddleHatchRoutine);
            }
        }
        if (m_Controller.GetYButtonPressed()) {
            if (elevatorBall) {
                m_Command.routines.push_back(m_TopBallRoutine);
            } else if (elevatorHatch) {
                m_Command.routines.push_back(m_TopHatchRoutine);
            } else {
                m_Command.hatchIntakeDown = true;
            }
        } else {
            m_Command.hatchIntakeDown = false;
        }
        if (m_Controller.GetXButtonPressed()) {
            m_Command.routines.push_back(m_TestRoutine);
        }
        if (elevatorDown) {
            m_RoutineManager->TerminateAllRoutines();
            m_Command.routines.push_back(m_LowerElevatorRoutine);
        } else if (elevatorSoftLand) {
            m_RoutineManager->TerminateAllRoutines();
            m_Elevator->SoftLand();
        }
        m_Command.elevatorInput = -m_Controller.GetY(frc::GenericHID::kLeftHand);
    }

    void Robot::TestInit() {
        Reset();
    }

    void Robot::TestPeriodic() {
        ControllablePeriodic();
    }
}

#ifndef RUNNING_FRC_TESTS

int main() {
    return frc::StartRobot<garage::Robot>();
}

#endif

#include <robot.hpp>

#include <lib/wait_routine.hpp>
#include <lib/sequential_routine.hpp>

#include <wpi/json.h>
#include <wpi/Path.h>
#include <wpi/FileSystem.h>
#include <wpi/raw_istream.h>

#include <frc/Filesystem.h>
#include <frc/DriverStation.h>

namespace garage {
    void Robot::RobotInit() {
        lib::Logger::Log(lib::Logger::LogLevel::k_Info, "Start robot initialization");
        auto begin = std::chrono::high_resolution_clock::now();
        m_Period = std::chrono::milliseconds(std::lround(m_period * 1000.0));
        // Setup network tables
        m_NetworkTableInstance = nt::NetworkTableInstance::GetDefault();
        m_NetworkTable = m_NetworkTableInstance.GetTable("Garage Robotics");
        m_DashboardNetworkTable = m_NetworkTable->GetSubTable("Dashboard");
        // Setup logging system
        const auto defaultLogLevel = lib::Logger::LogLevel::k_Verbose;
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
        // Create our routines
        CreateRoutines();
        // Find out how long initialization took and record it
        auto end = std::chrono::high_resolution_clock::now();
        auto initializationTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
        lib::Logger::Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("End robot initialization, took %d milliseconds", initializationTime));
    }

    void Robot::CreateRoutines() {
        // Render drive trajectory in initialization because it takes a couple of seconds
//        m_DriveForwardRoutine = std::make_shared<test::TestDriveAutoRoutine>(m_Pointer, "Test Drive");
//        m_DriveForwardRoutine->CalculatePath();
        m_ResetRoutine = std::make_shared<ResetRoutine>(m_Pointer);
        m_ResetWithServoRoutine = std::make_shared<ResetWithServoRoutine>(m_Pointer);
        /* Hatch routines */
        m_BottomHatchRoutine = std::make_shared<SetElevatorPositionRoutine>
                (m_Pointer, m_Config.bottomHatchHeight, "Bottom Hatch");
        // Rocket hatch routines
        m_RocketMiddleHatchRoutine = std::make_shared<SetElevatorPositionRoutine>
                (m_Pointer, m_Config.rocketMiddleHatchHeight, "Middle Rocket Hatch");
        m_RocketTopHatchRoutine = std::make_shared<SetElevatorPositionRoutine>
                (m_Pointer, m_Config.rocketTopHatchHeight, "Top Rocket Hatch");
        /* Ball routines */
        // Rocket
        m_RocketBottomBallRoutine = std::make_shared<SetElevatorPositionRoutine>
                (m_Pointer, m_Config.rocketBottomBallHeight, "Bottom Rocket Ball");
        m_RocketMiddleBallRoutine = std::make_shared<SetElevatorPositionRoutine>
                (m_Pointer, m_Config.rocketMiddleBallHeight, "Middle Rocket Ball");
        m_RocketTopBallRoutine = std::make_shared<SetElevatorPositionRoutine>
                (m_Pointer, m_Config.rocketTopBallHeight, "Top Rocket Ball");
        // Cargo
        m_CargoBallRoutine = std::make_shared<SetElevatorPositionRoutine>
                (m_Pointer, m_Config.cargoBallHeightDown, "Down Cargo Ball");
        /* Utility routines */
        m_FlipOverRoutine = std::make_shared<FlipOverRoutine>(m_Pointer);
        m_GroundBallIntakeRoutine = std::make_shared<BallIntakeRoutine>(m_Pointer, m_Config.groundIntakeBallHeight, FLIPPER_UPPER_ANGLE);
        m_LoadingBallIntakeRoutine = std::make_shared<BallIntakeRoutine>(m_Pointer, m_Config.loadingIntakeBallHeight, FLIPPER_LOWER_ANGLE);
        /* End game routines */
        m_EndGameRoutine = std::make_shared<LockFlipperRoutine>(m_Pointer);

        // Testing routine
        auto
                testWaitRoutineOne = std::make_shared<lib::WaitRoutine>(m_Pointer, 500l),
                testWaitRoutineTwo = std::make_shared<lib::WaitRoutine>(m_Pointer, 1000l);
//        m_TestRoutine = std::make_shared<lib::ParallelRoutine>
//                (m_Pointer, "Test Routine", lib::RoutineVector{testWaitRoutineOne, testWaitRoutineTwo});
        m_TestRoutine = std::make_shared<SetElevatorPositionRoutine>(m_Pointer, 15000, "Meme");
    }

    void Robot::ReadConfig() {
        wpi::SmallString<PATH_LENGTH> deployDirectory;
        frc::filesystem::GetDeployDirectory(deployDirectory);
        wpi::sys::path::append(deployDirectory, "settings.json");
        std::error_code errorCode;
        wpi::raw_fd_istream settingsFile(deployDirectory, errorCode);
        if (errorCode) {
            lib::Logger::Log(lib::Logger::LogLevel::k_Fatal,
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
                        auto ballRoutines = routinesJson.at("ball");
                        {
                            auto intakeRoutines = ballRoutines.at("intake");
                            m_Config.groundIntakeBallHeight = intakeRoutines.at("ground_set_point").get<int>();
                            m_Config.loadingIntakeBallHeight = intakeRoutines.at("loading_set_point").get<int>();
                        }
                        {
                            auto rocketRoutines = ballRoutines.at("rocket");
                            // Set Points
                            m_Config.rocketBottomBallHeight = rocketRoutines.at("bottom_set_point").get<int>();
                            m_Config.rocketMiddleBallHeight = rocketRoutines.at("middle_set_point").get<int>();
                            m_Config.rocketTopBallHeight = rocketRoutines.at("top_set_point").get<int>();
                            // Angles
                            m_Config.rocketBottomBallAngle = rocketRoutines.at("bottom_angle").get<double>();
                            m_Config.rocketMiddleBallAngle = rocketRoutines.at("middle_angle").get<double>();
                            m_Config.rocketTopBallAngle = rocketRoutines.at("top_angle").get<double>();
                        }
                        {
                            auto cargoRoutines = ballRoutines.at("cargo");
                            m_Config.cargoBallHeightUp = cargoRoutines.at("up_set_point").get<int>();
                            m_Config.cargoBallHeightDown = cargoRoutines.at("down_set_point").get<int>();
                        }
                    }
                    {
                        auto hatchRoutines = routinesJson.at("hatch");
                        m_Config.bottomHatchHeight = hatchRoutines.at("bottom").get<int>();
                        {
                            auto rocketRoutines = hatchRoutines.at("rocket");
                            m_Config.rocketMiddleHatchHeight = rocketRoutines.at("middle").get<int>();
                            m_Config.rocketTopHatchHeight = rocketRoutines.at("top").get<int>();
                        }
                    }
                }
            } catch (wpi::detail::parse_error& error) {
                lib::Logger::Log(lib::Logger::LogLevel::k_Fatal, lib::Logger::Format("Error parsing robot settings: %s", error.what()));
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
        /* Routines */
        m_Command.routines.clear();
        if (m_PrimaryController.GetBackButtonPressed()) {
            // TODO remove after testing
            m_Command.routines.push_back(m_TestRoutine);
        }
        if (m_PrimaryController.GetStickButtonPressed(frc::GenericHID::kRightHand)) {
            m_Command.drivePrecisionEnabled = !m_Command.drivePrecisionEnabled;
            m_DashboardNetworkTable->PutString("Drive Mode", m_Command.drivePrecisionEnabled ? "Precise" : "Coarse");
        }
        if (m_PrimaryController.GetStartButtonPressed()) {
            m_Command.offTheBooksModeEnabled = !m_Command.offTheBooksModeEnabled;
            m_DashboardNetworkTable->PutString("Off the Books", m_Command.drivePrecisionEnabled ? "Yeet" : "Naw");
            m_DashboardNetworkTable->PutNumber("Match Time Remaining", frc::DriverStation::GetInstance().GetMatchTime());
            m_RoutineManager->TerminateAllRoutines();
            if (m_Command.offTheBooksModeEnabled) {
                m_Command.drivePrecisionEnabled = true;
                m_Command.routines.push_back(m_EndGameRoutine);
            } else {
                m_Command.drivePrecisionEnabled = false;
                m_Command.routines.push_back(m_ResetWithServoRoutine);
            }
        }
        const int pov = m_PrimaryController.GetPOV();
        const bool
        // Left button
                elevatorHatch = pov == 90,
        // Button button
                elevatorDown = pov == 180,
        // Right button
                elevatorBall = pov == 270,
        // Top button
                elevatorSoftLand = pov == 0;
        if (!m_Command.offTheBooksModeEnabled) {
            if (m_PrimaryController.GetAButtonPressed()) {
//                if (elevatorBall) {
//                    m_Command.routines.push_back(m_RocketBottomBallRoutine);
//                } else if (elevatorHatch) {
//                    m_Command.routines.push_back(m_BottomHatchRoutine);
//                } else {
//                    m_Command.routines.push_back(m_GroundBallIntakeRoutine);
//                }
                m_Command.routines.push_back(m_TestRoutine);
            }
            if (m_PrimaryController.GetBButtonPressed()) {
                if (elevatorBall) {
                    m_Command.routines.push_back(m_RocketMiddleBallRoutine);
                } else if (elevatorHatch) {
                    m_Command.routines.push_back(m_RocketMiddleHatchRoutine);
                } else {
                    m_Command.routines.push_back(m_CargoBallRoutine);
                }
            }
            if (m_PrimaryController.GetYButtonPressed()) {
                if (elevatorBall) {
                    m_Command.routines.push_back(m_RocketTopBallRoutine);
                } else if (elevatorHatch) {
                    m_Command.routines.push_back(m_RocketTopHatchRoutine);
                } else {
                    m_Command.hatchIntakeDown = true;
                }
            } else {
                m_Command.hatchIntakeDown = false;
            }
            if (m_PrimaryController.GetXButtonPressed()) {
                m_RoutineManager->TerminateAllRoutines();
                m_Command.routines.push_back(m_FlipOverRoutine);
            }
            const int secondaryPov = m_SecondaryController.GetPOV();
            if (elevatorDown || secondaryPov == 180) {
                m_RoutineManager->TerminateAllRoutines();
                m_Command.routines.push_back(m_ResetRoutine);
            } else if (elevatorSoftLand || secondaryPov == 0) {
                m_RoutineManager->TerminateAllRoutines();
                m_Elevator->SoftLand();
            }
        }
        double angle = FLIPPER_UPPER_ANGLE;
        if (m_Flipper) {
            angle = m_Flipper->GetAngle();
        }
        const bool shouldInvertDrive = angle < FLIPPER_STOW_ANGLE;
        m_Command.driveForward = -m_PrimaryController.GetY(frc::GenericHID::JoystickHand::kRightHand);
        m_Command.driveTurn = m_PrimaryController.GetX(frc::GenericHID::JoystickHand::kRightHand);
        if (shouldInvertDrive) {
            m_Command.driveForward *= -1;
            m_Command.driveTurn *= -1;
        }
        m_Command.elevatorInput = -m_PrimaryController.GetY(frc::GenericHID::kLeftHand);
        double triggers = m_PrimaryController.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) -
                          m_PrimaryController.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
        triggers = math::clamp(triggers, -1.0, 1.0);
        auto bumpers = math::axis<double>(
                m_PrimaryController.GetBumper(frc::GenericHID::JoystickHand::kRightHand),
                m_PrimaryController.GetBumper(frc::GenericHID::JoystickHand::kLeftHand));
        bumpers = math::clamp(bumpers, -1.0, 1.0);
        if (m_Command.offTheBooksModeEnabled) {
            m_Command.outrigger = triggers;
            m_Command.outriggerWheel = bumpers;
            m_Command.ballIntake = 0;
            m_Command.flipper = 0;
        } else {
            m_Command.outrigger = 0;
            m_Command.outriggerWheel = 0;
            m_Command.ballIntake = bumpers;
            m_Command.flipper = triggers;
        }
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

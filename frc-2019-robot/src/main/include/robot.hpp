#pragma once

#include <command.hpp>
#include <subsystem/drive.hpp>
#include <subsystem/flipper.hpp>
#include <subsystem/elevator.hpp>
#include <subsystem/outrigger.hpp>
#include <subsystem/ball_intake.hpp>
#include <subsystem/hatch_intake.hpp>

#include <lib/logger.hpp>
#include <lib/routine.hpp>
#include <lib/subsystem.hpp>
#include <lib/routine_manager.hpp>
#include <test/test_drive_auto_routine.hpp>
#include <routine/reset_routine.hpp>
#include <routine/flip_over_routine.hpp>
#include <routine/ball_intake_routine.hpp>
#include <routine/lock_flipper_routine.hpp>
#include <routine/ball_placement_routine.hpp>
#include <routine/elevator_and_flipper_routine.hpp>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

#include <wpi/optional.h>

#include <chrono>
#include <memory>

#define PATH_LENGTH 256

namespace garage {
    struct RobotConfig {
        bool shouldOutput = true, enableElevator = true, enableDrive = true, enableFlipper = true, enableBallIntake = true, enableHatchIntake = true, enableOutrigger = true;
        int bottomBallHeight, middleBallHeight, topBallHeight, bottomHatchHeight, middleHatchHeight, topHatchHeight;
        double bottomBallAngle, middleBallAngle, topBallAngle;
    };

    class Robot : public frc::TimedRobot {
    private:
        std::shared_ptr<Robot> m_Pointer;

        void UpdateCommand();

    protected:
        nt::NetworkTableInstance m_NetworkTableInstance;
        std::shared_ptr<nt::NetworkTable> m_NetworkTable;
        frc::XboxController m_Controller{0};
//        frc::Joystick m_Joystick{0};
        Command m_Command;
        std::shared_ptr<lib::RoutineManager> m_RoutineManager;
        std::shared_ptr<Drive> m_Drive;
        std::shared_ptr<Flipper> m_Flipper;
        std::shared_ptr<Elevator> m_Elevator;
        std::shared_ptr<Outrigger> m_Outrigger;
        std::shared_ptr<BallIntake> m_BallIntake;
        std::shared_ptr<HatchIntake> m_HatchIntake;
        std::vector<std::shared_ptr<lib::Subsystem>> m_Subsystems;
        wpi::optional<std::chrono::system_clock::time_point> m_LastPeriodicTime;
        RobotConfig m_Config = RobotConfig();
        std::chrono::milliseconds m_Period;
        // Routines
//        std::shared_ptr<test::TestDriveAutoRoutine> m_DriveForwardRoutine;
        std::shared_ptr<lib::Routine>
                m_TestRoutine,
                m_ResetRoutine,
                m_BottomHatchRoutine, m_MiddleHatchRoutine, m_TopHatchRoutine,
                m_BottomBallRoutine, m_MiddleBallRoutine, m_TopBallRoutine,
                m_FlipOverRoutine, m_BallIntakeRoutine, m_CargoRoutine,
                m_EndGameRoutine;

    public:
        void RobotInit() override;

        void ReadConfig();

        void Reset();

        void RobotPeriodic() override;

        void DisabledInit() override;

        void DisabledPeriodic() override;

        void AutonomousInit() override;

        void AutonomousPeriodic() override;

        void TeleopInit() override;

        void TeleopPeriodic() override;

        void AddSubsystem(std::shared_ptr<lib::Subsystem> subsystem);

        void ControllablePeriodic();

        bool ShouldOutput() const {
            return m_Config.shouldOutput;
        }

        RobotConfig& GetConfig() {
            return m_Config;
        }

        Command GetLatestCommand() {
            return m_Command;
        }

        std::shared_ptr<NetworkTable> GetNetworkTable() const {
            return m_NetworkTable;
        }

        std::shared_ptr<Elevator> GetElevator() {
            return m_Elevator;
        }

        std::shared_ptr<Drive> GetDrive() {
            return m_Drive;
        }

        std::shared_ptr<Outrigger> GetOutrigger() {
            return m_Outrigger;
        }

        std::shared_ptr<BallIntake> GetBallIntake() {
            return m_BallIntake;
        }

        std::shared_ptr<Flipper> GetFlipper() {
            return m_Flipper;
        }

        std::shared_ptr<lib::RoutineManager> GetRoutineManager() {
            return m_RoutineManager;
        }

        void TestInit() override;

        void TestPeriodic() override;
    };
}

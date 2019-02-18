#pragma once

#include <command.hpp>
#include <subsystem/drive.hpp>
#include <subsystem/flipper.hpp>
#include <subsystem/elevator.hpp>
#include <subsystem/outrigger.hpp>
#include <subsystem/ball_intake.hpp>
#include <subsystem/hatch_intake.hpp>

#include <lib/logger.hpp>
#include <lib/subsystem.hpp>
#include <lib/routine_manager.hpp>

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <memory>

namespace garage {
    class Robot : public frc::TimedRobot {
    private:
        std::shared_ptr<Robot> m_Pointer;

        void UpdateCommand();

    protected:
        nt::NetworkTableInstance m_NetworkTableInstance;
        std::shared_ptr<nt::NetworkTable> m_NetworkTable;
        frc::XboxController m_Controller{0};
        Command m_Command;
        std::shared_ptr<lib::RoutineManager> m_RoutineManager;
        std::shared_ptr<lib::Logger> m_Logger;
        std::shared_ptr<Drive> m_Drive;
        std::shared_ptr<Flipper> m_Flipper;
        std::shared_ptr<Elevator> m_Elevator;
        std::shared_ptr<Outrigger> m_Outrigger;
        std::shared_ptr<BallIntake> m_BallIntake;
        std::shared_ptr<HatchIntake> m_HatchIntake;
        std::vector<std::shared_ptr<lib::Subsystem>> m_Subsystems;

    public:
        void RobotInit() override;

        void RobotPeriodic() override;

        void DisabledInit() override;

        void DisabledPeriodic() override;

        void AutonomousInit() override;

        void AutonomousPeriodic() override;

        void TeleopInit() override;

        void TeleopPeriodic() override;

        void AddSubsystem(std::shared_ptr<lib::Subsystem> subsystem);

        void ExecuteCommand();

        std::shared_ptr<NetworkTable> GetNetworkTable() const;

        std::shared_ptr<lib::Logger> GetLogger();

        std::shared_ptr<Elevator> GetElevator();

        std::shared_ptr<Drive> GetDrive();
    };
}

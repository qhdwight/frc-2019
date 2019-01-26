#pragma once

#include <thread>
#include <memory>

#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <frc/XboxController.h>
#include <frc/TimedRobot.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

#include <lib/subsystem.hpp>

#include <command.hpp>
#include <subsystem/drive.hpp>
#include <subsystem/ball_intake.hpp>

namespace garage {
    class Robot : public frc::TimedRobot {
    public:
        void RobotInit() override;

        void RobotPeriodic() override;

        void DisabledInit() override;

        void DisabledPeriodic() override;

        void AutonomousInit() override;

        void AutonomousPeriodic() override;

        void TeleopInit() override;

        void TeleopPeriodic() override;

        std::shared_ptr<NetworkTable> GetNetworkTable() const;

    protected:
        nt::NetworkTableInstance m_NetworkTableInstace;
        std::shared_ptr<nt::NetworkTable> m_NetworkTable;
        frc::XboxController m_Controller{0};
        std::shared_ptr<Drive> m_Drive = std::make_shared<Drive>();
        std::shared_ptr<BallIntake> m_BallIntake = std::make_shared<BallIntake>();
        std::vector<std::shared_ptr<lib::Subsystem>> m_Subsystems;

        Command GetCommand();

        static void VisionThread();
    };
}
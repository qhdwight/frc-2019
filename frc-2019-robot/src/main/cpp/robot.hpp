#pragma once

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

namespace robot {
    class Robot : public frc::TimedRobot {
    public:
        void RobotInit() override;

        void DisabledInit() override;

        void AutonomousInit() override;

        void AutonomousPeriodic() override;

        void TeleopInit() override;

        void TeleopPeriodic() override;

    protected:
        frc::Joystick m_Stick{0};
        ctre::phoenix::motorcontrol::can::TalonSRX m_RightSRX{0}, m_LeftSRX{1};
        ctre::phoenix::motorcontrol::can::VictorSPX m_RightSPX{2}, m_LeftSPX{3};
    };
}
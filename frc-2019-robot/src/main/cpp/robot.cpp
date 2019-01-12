#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

#include "ctre.hpp"

namespace rbt {
    class Robot : public frc::TimedRobot {
    public:
        void TeleopInit() override {
            m_LeftSPX.Follow(m_LeftSRX);
            m_RightSPX.Follow(m_RightSRX);
            m_LeftSPX.SetInverted(true);
        }

        void TeleopPeriodic() override {
            double
                    forward = -m_Stick.GetY(),
                    turn = m_Stick.GetX();
            m_LeftSRX.Set(ctr::ControlMode::PercentOutput, forward + turn * 2);
            m_RightSRX.Set(ctr::ControlMode::PercentOutput, forward - turn * 2);
        }

    private:
        frc::Joystick m_Stick{0};
        ctr::TalonSRX m_RightSRX{0}, m_LeftSRX{1};
        ctr::VictorSPX m_RightSPX{2}, m_LeftSPX{3};
    };
}

#ifndef RUNNING_FRC_TESTS

int main() {
    return frc::StartRobot<rbt::Robot>();
}

#endif

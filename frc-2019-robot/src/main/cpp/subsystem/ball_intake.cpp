#include <subsystem/ball_intake.hpp>

namespace garage {
    void BallIntake::ExecuteCommand(Command& command) {
        const double ballIntake = command.ballIntake;
        m_LeftIntakeSPX.Set(ballIntake);
        m_RightIntakeSPX.Set(ballIntake);
    }

    BallIntake::BallIntake() {
        m_LeftIntakeSPX.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_RightIntakeSPX.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    }
}

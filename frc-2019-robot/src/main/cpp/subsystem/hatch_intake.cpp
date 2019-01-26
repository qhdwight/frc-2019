#include <subsystem/hatch_intake.hpp>

namespace garage {
    void HatchIntake::Initialize() {
        m_Intake.ConfigFactoryDefault();
    }

    void HatchIntake::ExecuteCommand(Command& command) {
//    if (command.trigger) {
//        m_ChangeIntakeOpen = true;
//        const double current = m_IntakeOpen ? INTAKE_OUTPUT : -INTAKE_OUTPUT;
//        m_LeftIntakeSPX.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, current);
//    } else {
//        if (m_ChangeIntakeOpen)
//            m_IntakeOpen = !m_IntakeOpen;
//        m_ChangeIntakeOpen = false;
//        m_LeftIntakeSPX.StopMotor();
//    }
    }
}
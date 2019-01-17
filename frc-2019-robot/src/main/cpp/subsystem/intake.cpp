#include <subsystem/intake.hpp>

void Intake::ExecuteCommand(Command& command) {
//    if (command.trigger) {
//        m_ChangeIntakeOpen = true;
//        const double current = m_IntakeOpen ? INTAKE_OUTPUT : -INTAKE_OUTPUT;
//        m_IntakeSPX.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, current);
//    } else {
//        if (m_ChangeIntakeOpen)
//            m_IntakeOpen = !m_IntakeOpen;
//        m_ChangeIntakeOpen = false;
//        m_IntakeSPX.StopMotor();
//    }
}

Intake::Intake() : Subsystem() {

}

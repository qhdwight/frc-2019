#include <subsystem/hatch_intake.hpp>

namespace garage {
    HatchIntake::HatchIntake(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
        m_Servo.SetRaw(SERVO_LOW);
    }

    void HatchIntake::ExecuteCommand(Command& command) {
        if (command.hatchIntakeDown) {
            m_ServoOutput = m_IntakeOpen ? SERVO_LOW : SERVO_HIGH;
            m_IntakeOpen = !m_IntakeOpen;
        }
        m_Servo.SetRaw(m_ServoOutput);
//    if (command.trigger) {
//        m_ChangeIntakeOpen = true;
//        const double current = m_IntakeOpen ? INTAKE_OUTPUT : -INTAKE_OUTPUT;
//        m_LeftIntakeSPX.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, current);xz
//    } else {
//        if (m_ChangeIntakeOpen)
//            m_IntakeOpen = !m_IntakeOpen;
//        m_ChangeIntakeOpen = false;
//        m_LeftIntakeSPX.StopMotor();
//    }
    }
}
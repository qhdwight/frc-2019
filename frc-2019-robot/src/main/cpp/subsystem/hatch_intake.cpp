#include <subsystem/hatch_intake.hpp>

#include <garage_math/garage_math.hpp>

#include <robot.hpp>

namespace garage {
    HatchIntake::HatchIntake(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
        m_Servo.SetRaw(HATCH_SERVO_LOWER);
        m_Robot->GetNetworkTable()->PutNumber("Lower PWM", HATCH_SERVO_LOWER);
        m_Robot->GetNetworkTable()->PutNumber("Upper PWM", HATCH_SERVO_UPPER);
    }

    void HatchIntake::ExecuteCommand(Command& command) {
        if (command.hatchIntakeDown) {
            m_ServoOutput = static_cast<uint16_t>(m_IntakeOpen ? HATCH_SERVO_LOWER : HATCH_SERVO_UPPER);
            m_IntakeOpen = !m_IntakeOpen;
        }
        m_Servo.SetRaw(m_ServoOutput);
//        const double
//                lower = m_Robot->GetNetworkTable()->GetNumber("Lower PWM", HATCH_SERVO_LOWER),
//                upper = m_Robot->GetNetworkTable()->GetNumber("Upper PWM", HATCH_SERVO_UPPER),
//                output = math::map(command.flipper, -1.0, 1.0, lower, upper);
//        m_Robot->GetNetworkTable()->PutNumber("Hatch Servo PWM", output);
//        m_Servo.SetRaw(static_cast<uint16_t>(output));
        Subsystem::ExecuteCommand(command);
    }
}
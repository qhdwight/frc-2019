#include <subsystem/hatch_intake.hpp>

#include <garage_math/garage_math.hpp>

#include <robot.hpp>

namespace garage {
    HatchIntake::HatchIntake(std::shared_ptr<Robot>& robot) : lib::Subsystem(robot, "Hatch Intake") {

    }

    void HatchIntake::Reset() {
        Subsystem::Reset();
        m_IntakeOpen = false;
        m_ServoOutput = HATCH_SERVO_UPPER;
    }

    void HatchIntake::UpdateUnlocked(Command& command) {
        if (command.hatchIntakeDown) {
            m_ServoOutput = static_cast<uint16_t>(m_IntakeOpen ? HATCH_SERVO_LOWER : HATCH_SERVO_UPPER);
            m_IntakeOpen = !m_IntakeOpen;
        }
        LogSample(lib::Logger::LogLevel::k_Debug, lib::Logger::Format("Servo Output: %d", m_ServoOutput));
    }

    void HatchIntake::Update() {
        m_Servo.SetRaw(m_ServoOutput);
    }
}
#include <subsystem/outrigger.hpp>

#include <garage_math/garage_math.hpp>

namespace garage {

    Outrigger::Outrigger(std::shared_ptr<Robot>& robot) : lib::Subsystem(robot, "Outrigger") {
        m_OutriggerMaster.RestoreFactoryDefaults();
        m_OutriggerSlave.RestoreFactoryDefaults();
        m_OutriggerWheel.RestoreFactoryDefaults();
        m_OutriggerMaster.SetOpenLoopRampRate(OUTRIGGER_RAMPING);
        m_OutriggerWheel.SetOpenLoopRampRate(OUTRIGGER_RAMPING);
        m_OutriggerMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_OutriggerSlave.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_OutriggerSlave.Follow(m_OutriggerMaster, true);
        m_OutriggerMaster.EnableVoltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        m_OutriggerWheel.EnableVoltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
    }

    void Outrigger::Reset() {
        Subsystem::Reset();
        StopMotors();
    }

    void Outrigger::StopMotors() {
        m_OutriggerMaster.Set(0.0);
        m_OutriggerWheel.Set(0.0);
    }

    void Outrigger::UpdateUnlocked(Command& command) {
        m_Output = math::threshold(command.test, DEFAULT_INPUT_THRESHOLD);
    }

    void Outrigger::Update() {
        m_OutriggerWheel.Set(m_Output);
    }

    void Outrigger::SetOutput(double output) {
        m_Output = output;
    }
}
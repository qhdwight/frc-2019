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
        StopMotors();
    }

    void Outrigger::Reset() {
        Subsystem::Reset();
        StopMotors();
    }

    void Outrigger::StopMotors() {
        m_OutriggerOutput = 0.0;
        m_OutriggerWheelOutput = 0.0;
        m_OutriggerMaster.Set(0.0);
        m_OutriggerWheel.Set(0.0);
    }

    bool Outrigger::ShouldUnlock(Command& command) {
        return math::absolute(command.outrigger) > DEFAULT_INPUT_THRESHOLD ||
               math::absolute(command.outriggerWheel) > DEFAULT_INPUT_THRESHOLD;
    }

    void Outrigger::UpdateUnlocked(Command& command) {
        m_OutriggerOutput = math::threshold(command.outrigger, DEFAULT_INPUT_THRESHOLD);
        m_OutriggerWheelOutput = math::threshold(command.outriggerWheel, DEFAULT_INPUT_THRESHOLD);
    }

    void Outrigger::Update() {
        m_OutriggerMaster.Set(m_OutriggerOutput);
        m_OutriggerWheel.Set(m_OutriggerWheelOutput);
    }

    void Outrigger::SetOutput(double output) {
        Lock();
        m_OutriggerOutput = output;
    }
}
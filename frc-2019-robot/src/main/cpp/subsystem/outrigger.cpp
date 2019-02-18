#include <subsystem/outrigger.hpp>

namespace garage {

    Outrigger::Outrigger(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Outrigger") {
        m_OutriggerMaster.RestoreFactoryDefaults();
        m_OutriggerSlave.RestoreFactoryDefaults();
        m_OutriggerWheel.RestoreFactoryDefaults();
        m_OutriggerMaster.SetOpenLoopRampRate(1.0);
        m_OutriggerWheel.SetOpenLoopRampRate(1.0);
        m_OutriggerMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_OutriggerSlave.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_OutriggerSlave.Follow(m_OutriggerMaster, true);
    }

    void Outrigger::ExecuteCommand(Command& command) {
        m_OutriggerWheel.Set(command.test);
    }
}
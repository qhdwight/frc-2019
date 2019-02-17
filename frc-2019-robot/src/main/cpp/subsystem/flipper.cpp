#include <subsystem/flipper.hpp>

#include <robot.hpp>

namespace garage {
    Flipper::Flipper(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
        m_Flipper.RestoreFactoryDefaults();
        m_Flipper.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_Flipper.SetOpenLoopRampRate(0.5);
    }

    void Flipper::ExecuteCommand(Command& command) {
        const double output = command.flipper * 0.2;
        m_Flipper.Set(output);
        m_Robot->GetNetworkTable()->PutNumber("Flipper/Flipper Amperage", m_Flipper.GetOutputCurrent());
        m_Robot->GetNetworkTable()->PutNumber("Flipper/Flipper Output", output);
        Subsystem::ExecuteCommand(command);
    }
}
#include <subsystem/flipper.hpp>

#include <robot.hpp>

namespace garage {
    Flipper::Flipper(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
//        m_Flipper.RestoreFactoryDefaults();
        m_Flipper.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }

    void Flipper::ExecuteCommand(Command& command) {
        const double output = command.flipper * 0.5;
        m_Flipper.Set(output);
        m_Robot->GetNetworkTable()->PutNumber("Flipper Amperage", m_Flipper.GetOutputCurrent());
        m_Robot->GetNetworkTable()->PutNumber("Flipper Output", output);
        Subsystem::ExecuteCommand(command);
    }
}
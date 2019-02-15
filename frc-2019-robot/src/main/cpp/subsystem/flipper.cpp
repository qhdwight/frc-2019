#include <subsystem/flipper.hpp>

#include <robot.hpp>

namespace garage {
    Flipper::Flipper(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
//        m_Flipper.RestoreFactoryDefaults();
        m_Flipper.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }

    void Flipper::ExecuteCommand(Command& command) {
        const double flipper = command.flipper;
        m_Flipper.Set(flipper * 0.5);
        m_Robot->GetNetworkTable()->PutNumber("Flipper Amperage", m_Flipper.GetOutputCurrent());
        Subsystem::ExecuteCommand(command);
    }
}
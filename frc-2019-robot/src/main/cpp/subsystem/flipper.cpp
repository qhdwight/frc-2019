#include <subsystem/flipper.hpp>

#include <robot.hpp>

namespace garage {
    Flipper::Flipper(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
        m_Flipper.ConfigFactoryDefault();
    }

    void Flipper::ExecuteCommand(Command& command) {
        const double flipper = command.flipper;
//        m_Flipper.ConfigOpenloopRamp()
        m_Flipper.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_Flipper.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, flipper * 0.5);
        m_Robot->GetNetworkTable()->PutNumber("Flipper Amperage", m_Flipper.GetOutputCurrent());
    }
}
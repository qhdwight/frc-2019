#include <subsystem/flipper.hpp>

namespace garage {
    void Flipper::Initialize() {
        m_Flipper.ConfigFactoryDefault();
    }

    void Flipper::ExecuteCommand(Command& command) {
        const double flipper = command.flipper;
//        m_Flipper.ConfigOpenloopRamp()
        m_Flipper.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, flipper * 0.25);
    }
}
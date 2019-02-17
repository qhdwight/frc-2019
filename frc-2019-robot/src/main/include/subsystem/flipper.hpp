#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <rev/CANSparkMax.h>

namespace garage {
    class Flipper : public lib::Subsystem {
    private:
        rev::CANSparkMax m_Flipper{FLIPPER, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANPIDController m_FlipperController = m_Flipper.GetPIDController();
        rev::CANEncoder m_Encoder = m_Flipper.GetEncoder();
        rev::CANDigitalInput m_LimitSwitch = m_Flipper.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
    public:
        Flipper(std::shared_ptr<Robot>& robot);

        void ExecuteCommand(Command& command) override;
    };
}

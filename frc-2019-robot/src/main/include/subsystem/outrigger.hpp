#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <rev/CANSparkMax.h>

namespace garage {
    class Outrigger : public lib::Subsystem {
    private:
        rev::CANSparkMax m_Outrigger{OUTRIGGER_ARM, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANPIDController m_OutriggerController = m_Outrigger.GetPIDController();
        rev::CANEncoder m_Encoder = m_Outrigger.GetEncoder();
    public:
        Outrigger(std::shared_ptr<Robot>& robot);

        void ExecuteCommand(Command& command) override;
    };
}
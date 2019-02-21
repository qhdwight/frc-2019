#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <rev/CANSparkMax.h>

namespace garage {
    class Outrigger : public lib::Subsystem {
    private:
        double m_Output = 0.0;
        rev::CANSparkMax
                m_OutriggerMaster{OUTRIGGER_ARM_MASTER, rev::CANSparkMax::MotorType::kBrushless},
                m_OutriggerSlave{OUTRIGGER_ARM_SLAVE, rev::CANSparkMax::MotorType::kBrushless},
                m_OutriggerWheel{OUTRIGGER_WHEEL, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANPIDController m_Controller = m_OutriggerMaster.GetPIDController();
        rev::CANEncoder m_Encoder = m_OutriggerMaster.GetEncoder();

    protected:
        void ProcessCommand(Command& command) override;

        void Update() override;

    public:
        Outrigger(std::shared_ptr<Robot>& robot);

        void SetOutput(double output);
    };
}
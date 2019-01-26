#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#include <lib/subsystem.hpp>

namespace garage {
    class HatchIntake : public lib::Subsystem {
    private:
        bool m_IntakeOpen, m_ChangeIntakeOpen;
        ctre::phoenix::motorcontrol::can::VictorSPX m_Intake{5};
    public:
        void Initialize() override;

        void ExecuteCommand(Command& command) override;
    };
}

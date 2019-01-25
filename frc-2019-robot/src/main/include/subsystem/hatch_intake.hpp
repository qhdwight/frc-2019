#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <lib/subsystem.hpp>

namespace garage {
    class HatchIntake : public lib::Subsystem {
    private:
        bool m_IntakeOpen, m_ChangeIntakeOpen;
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_IntakeSPX{5};
    public:
        void ExecuteCommand(Command& command) override;
    };
}

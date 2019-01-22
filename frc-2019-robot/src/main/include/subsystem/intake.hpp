#pragma once

#define INTAKE_OUTPUT 0.8

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

namespace garage {
    class Intake : public lib::Subsystem {
    private:
        bool m_IntakeOpen, m_ChangeIntakeOpen;
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_IntakeSPX{4};
    public:
        Intake();

        void ExecuteCommand(Command& command) override;
    };
}

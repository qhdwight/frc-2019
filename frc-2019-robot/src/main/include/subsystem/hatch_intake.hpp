#pragma once

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

namespace garage {
    class HatchIntake : public lib::Subsystem {
    private:
        bool m_IntakeOpen, m_ChangeIntakeOpen;
        ctre::phoenix::motorcontrol::can::VictorSPX m_Intake{5};
    public:
        HatchIntake(std::shared_ptr<Robot>& robot) : Subsystem(robot) {}

        void Initialize() override;

        void ExecuteCommand(Command& command) override;
    };
}

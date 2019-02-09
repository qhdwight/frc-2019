#pragma once

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

namespace garage {
    class BallIntake : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::VictorSPX m_LeftIntake{6}, m_RightIntake{7};
    public:
        BallIntake(std::shared_ptr<Robot>& robot);

        void ExecuteCommand(Command& command) override;
    };
}

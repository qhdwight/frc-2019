#pragma once

#define JOYSTICK_THRESHOLD 0.1

#include "../lib/subsystem.hpp"

#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

namespace garage {
    class Drive : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_RightSRX{0}, m_LeftSRX{1};
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_RightSPX{5}, m_LeftSPX{3};
        ctre::phoenix::sensors::PigeonIMU m_Pigeon{6};
    public:
        Drive();

        void ExecuteCommand(Command& command) override;
    };
}


#pragma once

#define JOYSTICK_THRESHOLD 0.1

#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <garage_math/garage_math.hpp>

#include <lib/subsystem.hpp>

#include <hardware_map.hpp>

namespace garage {
    class Drive : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_RightSRX{DRIVE_RIGHT_SRX}, m_LeftSRX{DRIVE_LEFT_SRX};
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_RightSPX{DRIVE_RIGHT_SPX}, m_LeftSPX{DRIVE_LEFT_SPX};
        ctre::phoenix::sensors::PigeonIMU m_Pigeon{PIGEON_IMU};
    public:
        Drive();

        void ExecuteCommand(Command& command) override;
    };
}


#pragma once

#define JOYSTICK_THRESHOLD 0.1

#include <memory>

#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#include <garage_math/garage_math.hpp>

#include <lib/subsystem.hpp>
#include <lib/pose_estimator.hpp>

#include <hardware_map.hpp>

namespace garage {
    class Drive : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::TalonSRX m_RightMaster{DRIVE_RIGHT_MASTER}, m_LeftMaster{DRIVE_LEFT_MASTER};
        ctre::phoenix::motorcontrol::can::VictorSPX m_RightSlave{DRIVE_RIGHT_SLAVE}, m_LeftSlave{DRIVE_LEFT_SLAVE};
        ctre::phoenix::sensors::PigeonIMU m_Pigeon{PIGEON_IMU};
        std::shared_ptr<lib::PoseEstimator> m_PoseEstimator;
    public:
        void Initialize() override;

        void ExecuteCommand(Command& command) override;
    };
}


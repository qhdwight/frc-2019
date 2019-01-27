#pragma once

#define JOYSTICK_THRESHOLD 0.1

#include <command.hpp>
#include <hardware_map.hpp>

#include <lib/subsystem.hpp>
#include <lib/pose_estimator.hpp>

#include <garage_math/garage_math.hpp>

#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#include <memory>

namespace garage {
    class Robot;

    class Drive : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::TalonSRX m_RightMaster{DRIVE_RIGHT_MASTER}, m_LeftMaster{DRIVE_LEFT_MASTER};
        ctre::phoenix::motorcontrol::can::VictorSPX m_RightSlave{DRIVE_RIGHT_SLAVE}, m_LeftSlave{DRIVE_LEFT_SLAVE};
        ctre::phoenix::sensors::PigeonIMU m_Pigeon{PIGEON_IMU};
        std::shared_ptr<lib::PoseEstimator> m_PoseEstimator;
    public:
        Drive(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
            Initialize();
        }

        void Initialize() override;

        void TeleopInit() override;

        void ExecuteCommand(Command& command) override;
    };
}


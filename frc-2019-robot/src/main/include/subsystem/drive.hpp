#pragma once

#define JOYSTICK_THRESHOLD 0.06

#include <command.hpp>
#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <garage_math/garage_math.hpp>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>

#include <memory>

namespace garage {
    class Drive : public lib::Subsystem {
    private:
        double m_LeftOutput, m_RightOutput;
        rev::CANSparkMax
                m_RightMaster{DRIVE_RIGHT_MASTER, rev::CANSparkMax::MotorType::kBrushless},
                m_LeftMaster{DRIVE_LEFT_MASTER, rev::CANSparkMax::MotorType::kBrushless},
                m_RightSlave{DRIVE_RIGHT_SLAVE, rev::CANSparkMax::MotorType::kBrushless},
                m_LeftSlave{DRIVE_LEFT_SLAVE, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANEncoder m_LeftEncoder = m_LeftMaster.GetEncoder(), m_RightEncoder = m_RightMaster.GetEncoder();
        ctre::phoenix::sensors::PigeonIMU m_Pigeon{PIGEON_IMU};
    public:
        Drive(std::shared_ptr<Robot>& robot);

        void TeleopInit() override;

        double InputFromCommand(double commandInput);

        void ExecuteCommand(Command& command) override;

        void SpacedUpdate(Command& command) override;

        double GetHeading();

        double GetTilt();

        void ResetGyroAndEncoders();

        void SetDriveOutput(double left, double right);
    };
}


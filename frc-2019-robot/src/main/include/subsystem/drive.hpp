#pragma once

#include <command.hpp>
#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <garage_math/garage_math.hpp>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>

#include <memory>

#define DRIVE_RAMPING 0.2

namespace garage {
    class Drive : public lib::Subsystem {
    private:
        double m_LeftOutput = 0.0, m_RightOutput = 0.0;
        double m_RightEncoderPosition = 0.0, m_LeftEncoderPosition = 0.0;
        rev::CANSparkMax
                m_RightMaster{DRIVE_RIGHT_MASTER, rev::CANSparkMax::MotorType::kBrushless},
                m_LeftMaster{DRIVE_LEFT_MASTER, rev::CANSparkMax::MotorType::kBrushless},
                m_RightSlave{DRIVE_RIGHT_SLAVE, rev::CANSparkMax::MotorType::kBrushless},
                m_LeftSlave{DRIVE_LEFT_SLAVE, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANEncoder m_LeftEncoder = m_LeftMaster.GetEncoder(), m_RightEncoder = m_RightMaster.GetEncoder();
        ctre::phoenix::sensors::PigeonIMU m_Pigeon{PIGEON_IMU};

    protected:
        double InputFromCommand(double commandInput);

        void SpacedUpdate(Command& command) override;

        void UpdateUnlocked(Command& command) override;

        void Update() override;

    public:
        Drive(std::shared_ptr<Robot>& robot);

        double GetHeading();

        double GetTilt();

        int GetDiscreteRightEncoderTicks();

        int GetDiscreteLeftEncoderTicks();

        void ResetGyroAndEncoders();

        void SetDriveOutput(double left, double right);
    };
}


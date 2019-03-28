#pragma once

#include <command.hpp>
#include <hardware_map.hpp>

#include <lib/limelight.hpp>
#include <lib/controllable_subsystem.hpp>

#include <garage_math/garage_math.hpp>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <memory>

#define DRIVE_RAMPING 0.12
#define DRIVE_INPUT_DEAD_BAND 0.02
#define DRIVE_TURN_POWER 0.4


#define DRIVE_NEGATIVE_INERTIA_THRESHOLD 0.65
#define DRIVE_NEGATIVE_INERTIA_TURN_SCALAR 0.005
#define DRIVE_NEGATIVE_INERTIA_CLOSE_SCALAR 0.01
#define DRIVE_NEGATIVE_INERTIA_FAR_SCALAR 0.02
#define DRIVE_SENSITIVITY 0.65
#define DRIVE_TURN_NON_LINEARITY 0.5

#define DRIVE_QUICK_STOP_DEAD_BAND 0.5
#define DRIVE_QUICK_STOP_WEIGHT 0.1
#define DRIVE_QUICK_STOP_SCALAR 5.0


#define VISION_LIMELIGHT_TABLE_NAME "limelight"

#define VISION_MAX_FORWARD 0.1
#define VISION_MAX_TURN 0.06

#define VISION_FORWARD_P 0.004
#define VISION_TURN_P 0.002
#define VISION_DESIRED_TARGET_AREA 29.0
#define VISION_AREA_THRESHOLD 4.0

namespace garage {
    class Drive;

    using DriveController=lib::SubsystemController<Drive>;

    class RawDriveController : public DriveController {
    public:
        RawDriveController(std::weak_ptr<Drive>& drive) : DriveController(drive, "Raw Drive Controller") {}

        void SetDriveOutput(double leftOutput, double rightOutput) {
            m_LeftOutput = leftOutput;
            m_RightOutput = rightOutput;
        }

    protected:
        double m_LeftOutput = 0.0, m_RightOutput = 0.0;

        void Control() override;

        void Reset() override;
    };

    class ManualDriveController : public DriveController {
    public:
        ManualDriveController(std::weak_ptr<Drive>& drive) : DriveController(drive, "Manual Drive Controller") {}

    protected:
        double m_ForwardInput = 0.0, m_TurnInput = 0.0, m_OldTurnInput = 0.0, m_QuickStopAccumulator = 0.0, m_NegativeInertiaAccumulator = 0.0;
        bool m_IsQuickTurn = false;

        void ProcessCommand(Command& command) override;

        void Control() override;

        void Reset() override;
    };

    class AutoAlignDriveController : public DriveController {
    public:
        AutoAlignDriveController(std::weak_ptr<Drive>& drive);

    protected:
        lib::Limelight& m_Limelight;

        void Control() override;
    };

    class Drive : public lib::ControllableSubsystem<Drive> {
        friend class RawDriveController;

        friend class ManualDriveController;

        friend class AutoAlignDriveController;

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
        std::shared_ptr<RawDriveController> m_RawController;
        std::shared_ptr<ManualDriveController> m_ManualController;
        std::shared_ptr<AutoAlignDriveController> m_AutoAlignController;

    protected:
        void SpacedUpdate(Command& command) override;

        void Update() override;

        bool ShouldUnlock(Command& command) override;

    public:
        Drive(std::shared_ptr<Robot>& robot);

        void OnPostInitialize() override;

        void StopMotors();

        void AutoAlign();

        double GetHeading();

        double GetTilt();

        int GetDiscreteRightEncoderTicks();

        int GetDiscreteLeftEncoderTicks();

        void ResetGyroAndEncoders();

        void SetDriveOutput(double left, double right);

        void Reset() override;
    };
}


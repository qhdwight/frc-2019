#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem_controller.hpp>
#include <lib/controllable_subsystem.hpp>

#include <frc/Servo.h>

#include <rev/CANSparkMax.h>

#define FLIPPER_SET_POINT_LOWER 3.0
#define FLIPPER_SET_POINT_UPPER 38.0

#define FLIPPER_LOWER 0.0
#define FLIPPER_UPPER 40.0
#define FLIPPER_LOWER_ANGLE 0.0
#define FLIPPER_UPPER_ANGLE 180.0

#define FLIPPER_P 4e-5
#define FLIPPER_I 1e-7
#define FLIPPER_D 7e-4
#define FLIPPER_I_ZONE 3.0
#define FLIPPER_FF 0.000156
#define FLIPPER_VELOCITY 4400.0
#define FLIPPER_ACCELERATION 2300.0
#define FLIPPER_ALLOWABLE_ERROR 0.0
#define FLIPPER_ANGLE_FF 0.05

#define FLIPPER_CLOSED_LOOP_RAMP 0.1
#define FLIPPER_SMART_MOTION_PID_SLOT 0

#define CAMERA_SERVO_LOWER 300
#define CAMERA_SERVO_UPPER 1800

#define LOCK_SERVO_LOWER 500
#define LOCK_SERVO_UPPER 1500

namespace garage {
    class Flipper;

    using FlipperController = lib::SubsystemController<Flipper>;

    class RawFlipperController : public FlipperController {
    private:
        double m_Input = 0.0, m_Output = 0.0;

    public:
        RawFlipperController(std::weak_ptr<Flipper>& subsystem)
                : SubsystemController(subsystem, "Raw Controller") {}

        void Reset() override;

        void ProcessCommand(Command& command) override;

        void Control() override;
    };

    class VelocityFlipperController : public FlipperController {
    private:
        double m_WantedVelocity = 0.0, m_Input = 0.0;

    public:
        VelocityFlipperController(std::weak_ptr<Flipper>& subsystem)
                : SubsystemController(subsystem, "Velocity Controller") {}

        void SetWantedVelocity(double velocity) {
            m_WantedVelocity = velocity;
        }

        void Reset() override;

        void ProcessCommand(Command& command) override;

        void Control() override;
    };

    class SetPointFlipperController : public FlipperController {
    private:
        double m_SetPoint = 0.0;

    public:
        SetPointFlipperController(std::weak_ptr<Flipper>& subsystem)
                : SubsystemController(subsystem, "Set Point Controller") {}

        void SetSetPoint(double setPoint) {
            m_SetPoint = setPoint;
        }

        void Reset() override;

        void ProcessCommand(Command& command) override;

        void Control() override;
    };

    class Flipper : public lib::ControllableSubsystem<Flipper> {
        friend class RawFlipperController;

        friend class VelocityFlipperController;

        friend class SetPointFlipperController;

    protected:
        rev::CANSparkMax m_FlipperMaster{FLIPPER, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANPIDController m_FlipperController = m_FlipperMaster.GetPIDController();
        rev::CANEncoder m_Encoder = m_FlipperMaster.GetEncoder();
        rev::CANDigitalInput m_LimitSwitch = m_FlipperMaster.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen);
        bool m_IsLimitSwitchDown = true, m_FirstLimitSwitchHit = true;
        double m_EncoderPosition = 0.0, m_EncoderVelocity = 0.0, m_Angle = 0.0;
        double m_AngleFeedForward = FLIPPER_ANGLE_FF, m_MaxVelocity = FLIPPER_VELOCITY;
        std::shared_ptr<RawFlipperController> m_RawController;
        std::shared_ptr<SetPointFlipperController> m_SetPointController;
        std::shared_ptr<VelocityFlipperController> m_VelocityController;
        frc::Servo m_CameraServo{CAMERA_SERVO}, m_LockServo{LOCK_SERVO};
        uint16_t m_CameraServoOutput = CAMERA_SERVO_LOWER, m_LockServoOutput = LOCK_SERVO_LOWER;

        void Update() override;

        void SpacedUpdate(Command& command) override;

        bool ShouldUnlock(Command& command) override;

    public:
        Flipper(std::shared_ptr<Robot>& robot);

        void OnPostInitialize() override;

        void SetRawOutput(double output);

        void SetSetPoint(double setPoint);

        void SetAngle(double angle);

        void LockServo();

        bool WithinAngle(double angle);

        double GetAngle();

        void Stow();

        double RawSetPointToAngle(double setPoint);

        double AngleToRawSetPoint(double angle);

        void Reset() override;
    };
}

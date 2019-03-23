#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem_controller.hpp>
#include <lib/controllable_subsystem.hpp>

#include <frc/Servo.h>

#include <rev/CANSparkMax.h>

#define FLIPPER_LOWER 0.0
#define FLIPPER_UPPER 40.0
#define FLIPPER_LOWER_ANGLE 0.0 // Degrees
#define FLIPPER_STOW_ANGLE 90.0 // Degrees
#define FLIPPER_UPPER_ANGLE 180.0 // Degrees

#define FLIPPER_COM_ANGLE_FF_OFFSET 25.0

#define FLIPPER_SET_POINT_THRESHOLD 1.0
#define FLIPPER_SET_POINT_LOWER (FLIPPER_LOWER + FLIPPER_SET_POINT_THRESHOLD) // Raw set point
#define FLIPPER_SET_POINT_UPPER (FLIPPER_UPPER - FLIPPER_SET_POINT_THRESHOLD) // Raw set point

//#define FLIPPER_P 0.000015
#define FLIPPER_P 0.0001
#define FLIPPER_I 0.0
#define FLIPPER_D 0.0
#define FLIPPER_MAX_ACCUM 0.0
#define FLIPPER_I_ZONE 0.0
#define FLIPPER_FF 0.00012
//#define FLIPPER_FF 0.00009
#define FLIPPER_VELOCITY 4500.0
#define FLIPPER_ACCELERATION 4500.0
#define FLIPPER_ALLOWABLE_ERROR 0.0
#define FLIPPER_ANGLE_FF 0.0335

#define FLIPPER_WITHIN_RANGE 2.0 // Degrees

#define FLIPPER_RAW_POWER 0.3
#define FLIPPER_MANUAL_POWER 0.6

#define FLIPPER_CLOSED_LOOP_RAMP 0.1
#define FLIPPER_SMART_MOTION_PID_SLOT 0

#define CAMERA_SERVO_LOWER 1700
#define CAMERA_SERVO_UPPER 400

#define LOCK_SERVO_LOWER 1500
#define LOCK_SERVO_UPPER 500

namespace garage {
    class Flipper;

    class FlipperController : public lib::SubsystemController<Flipper> {
    public:
        FlipperController(std::weak_ptr<Flipper>& subsystem, const std::string& name)
                : SubsystemController(subsystem, name) {}

        double GetWantedAngle();
    };

    class RawFlipperController : public FlipperController {
    private:
        double m_Output = 0.0;

    public:
        RawFlipperController(std::weak_ptr<Flipper>& subsystem)
                : FlipperController(subsystem, "Raw Controller") {}

        void Reset() override;

        void SetOutput(double output);

        void ProcessCommand(Command& command) override;

        void Control() override;
    };

    class VelocityFlipperController : public FlipperController {
    private:
        double m_WantedVelocity = 0.0;

    public:
        VelocityFlipperController(std::weak_ptr<Flipper>& subsystem)
                : FlipperController(subsystem, "Velocity Controller") {}

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
                : FlipperController(subsystem, "Set Point Controller") {}

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
        rev::CANDigitalInput
                m_ForwardLimitSwitch = m_FlipperMaster.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen),
                m_ReverseLimitSwitch = m_FlipperMaster.GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen);
        bool
                m_IsForwardLimitSwitchDown = false, m_FirstForwardLimitSwitchHit = true,
                m_IsReverseLimitSwitchDown = false, m_FirstReverseLimitSwitchHit = true;
        double m_EncoderPosition = 0.0, m_EncoderVelocity = 0.0, m_Angle = 0.0;
        double m_AngleFeedForward = FLIPPER_ANGLE_FF, m_MaxVelocity = FLIPPER_VELOCITY;
        std::shared_ptr<RawFlipperController> m_RawController;
        std::shared_ptr<SetPointFlipperController> m_SetPointController;
        std::shared_ptr<VelocityFlipperController> m_VelocityController;
        frc::Servo m_CameraServo{CAMERA_SERVO}, m_LockServo{LOCK_SERVO};
        uint16_t m_CameraServoOutput = CAMERA_SERVO_LOWER, m_LockServoOutput = LOCK_SERVO_LOWER;

        void SetupNetworkTableValues();

        void Update() override;

        void UpdateUnlocked(Command& command) override;

        void SpacedUpdate(Command& command) override;

        bool ShouldUnlock(Command& command) override;

        bool IsWithinMotorOutputConditions(double wantedOutput, double forwardThreshold, double reverseThreshold);

        void HandleLimitSwitch(rev::CANDigitalInput& limitSwitch, bool& isLimitSwitchDown, bool& isFirstHit, double resetEncoderValue);

    public:
        Flipper(std::shared_ptr<Robot>& robot);

        void OnPostInitialize() override;

        void SetRawOutput(double output);

        void SetSetPoint(double setPoint);

        void SetAngle(double angle);

        void LockServo();

        void UnlockServo();

        bool WithinAngle(double angle);

        double GetAngle();

        double GetWantedAngle();

        void Stow();

        double RawSetPointToAngle(double setPoint);

        double AngleToRawSetPoint(double angle);

        void Reset() override;
    };
}

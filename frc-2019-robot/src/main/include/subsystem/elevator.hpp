#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem_controller.hpp>
#include <lib/controllable_subsystem.hpp>

#include <rev/CANSparkMax.h>

#define ELEVATOR_MAX 110.0 // Encoder ticks
#define ELEVATOR_MIN 0.0 // Encoder ticks

/* Gains and Motion Magic */
#define ELEVATOR_VELOCITY 2500.0
#define ELEVATOR_ACCELERATION 2200.0
#define ELEVATOR_P 0.00002
#define ELEVATOR_I 0.0
#define ELEVATOR_MAX_ACCUM 0.0
#define ELEVATOR_I_ZONE 0 // Encoder ticks
#define ELEVATOR_D 0.0
//#define ELEVATOR_D ELEVATOR_P * 3.3
#define ELEVATOR_F 0.0002
#define ELEVATOR_FF 0.3
#define ELEVATOR_MAX_CLOSED_LOOP_HEIGHT (ELEVATOR_MAX - 0)
#define ELEVATOR_MIN_CLOSED_LOOP_HEIGHT 2.0 // Encoder ticks
#define ELEVATOR_MIN_RAW_HEIGHT 0 // Encoder ticks

/* Energy Management */
#define ELEVATOR_CONTINUOUS_CURRENT_LIMIT 40 // Amperes
#define ELEVATOR_PEAK_CURRENT_LIMIT 300 // Amperes
#define ELEVATOR_PEAK_CURRENT_DURATION 200 // Milliseconds

#define ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR 0 // Encoder ticks - Zero is always try to get to value and do not stop
#define ELEVATOR_WITHIN_SET_POINT_AMOUNT 2.0

#define ELEVATOR_OPEN_LOOP_RAMP 0.2 // Seconds
#define ELEVATOR_CLOSED_LOOP_RAMP 0.1 // Seconds

#define ELEVATOR_SAFE_DOWN 0.0 // Percent output

#define ELEVATOR_CLOSED_LOOP_SLOT 0

#define ELEVATOR_LAND_TIME 3.0 // Seconds left in match when elevator tries to land to avoid damage

namespace garage {
    class Elevator;

    class RawElevatorController : public lib::SubsystemController<Elevator> {
    protected:
        double m_Input = 0.0, m_Output = 0.0;

    public:
        RawElevatorController(std::weak_ptr<Elevator>& subsystem) : lib::SubsystemController<Elevator>(subsystem, "Raw Controller") {}

        void Reset() override;

        void ProcessCommand(Command& command) override;

        void Control() override;

        void SetRawOutput(double output) {
            m_Output = output;
        }
    };

    class SetPointElevatorController : public lib::SubsystemController<Elevator> {
    protected:
        double m_WantedSetPoint = 0;

    public:
        SetPointElevatorController(std::weak_ptr<Elevator>& subsystem) : lib::SubsystemController<Elevator>(subsystem, "Set Point Controller") {}

        void ProcessCommand(Command& command) override;

        void Control() override;

        void Reset() override;

        void SetWantedSetPoint(double wantedSetPoint) {
            m_WantedSetPoint = wantedSetPoint;
        }
    };

    class VelocityElevatorController : public lib::SubsystemController<Elevator> {
    protected:
        double m_Input = 0.0, m_WantedVelocity = 0.0;

    public:
        VelocityElevatorController(std::weak_ptr<Elevator>& subsystem) : lib::SubsystemController<Elevator>(subsystem, "Velocity Controller") {}

        void ProcessCommand(Command& command) override;

        void Control() override;

        void Reset() override;
    };

    class SoftLandElevatorController : public lib::SubsystemController<Elevator> {
    public:
        SoftLandElevatorController(std::weak_ptr<Elevator>& subsystem) : lib::SubsystemController<Elevator>(subsystem, "Soft Land Controller") {}

        void Control() override;
    };

    class Elevator : public lib::ControllableSubsystem<Elevator> {
        friend class RawElevatorController;

        friend class SetPointElevatorController;

        friend class VelocityElevatorController;

        friend class SoftLandElevatorController;

    protected:
        double m_EncoderPosition = 0, m_EncoderVelocity = 0, m_FeedForward = ELEVATOR_FF, m_MaxVelocity = ELEVATOR_VELOCITY;
        rev::CANSparkMax
                m_SparkMaster{ELEVATOR_MASTER, rev::CANSparkMax::MotorType::kBrushless},
                m_SparkSlave{ELEVATOR_SLAVE, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANPIDController m_SparkController = m_SparkMaster.GetPIDController();
        rev::CANEncoder m_Encoder = m_SparkSlave.GetEncoder();
        rev::CANDigitalInput m_ReverseLimitSwitch = m_SparkSlave.GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen);
        bool m_IsFirstLimitSwitchHit = true;
        std::shared_ptr<RawElevatorController> m_RawController;
        std::shared_ptr<SetPointElevatorController> m_SetPointController;
        std::shared_ptr<VelocityElevatorController> m_VelocityController;
        std::shared_ptr<SoftLandElevatorController> m_SoftLandController;

        void ConfigSpeedControllers();

        void SetupNetworkTableEntries();

        bool ShouldUnlock(Command& command) override;

        void Update() override;

        void UpdateUnlocked(Command& command) override;

        void SpacedUpdate(Command& command) override;

    public:
        Elevator(std::shared_ptr<Robot>& robot);

        void OnPostInitialize() override;

        bool WithinPosition(double targetPosition);

        void SetWantedSetPoint(double wantedSetPoint);

        void SetRawOutput(double output);

        void SoftLand();

        void ResetEncoder();

        double GetPosition() {
            return m_EncoderPosition;
        }

        double GetVelocity() {
            return m_EncoderVelocity;
        }

        void Reset() override;
    };
}

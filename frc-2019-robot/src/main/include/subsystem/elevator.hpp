#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem_controller.hpp>
#include <lib/controllable_subsystem.hpp>

#include <rev/CANSparkMax.h>

#define ELEVATOR_MAX 110.0 // Encoder ticks
#define ELEVATOR_MIN 0.0 // Encoder ticks

/* Gains and Motion Magic */
// Normal
#define ELEVATOR_VELOCITY 5500.0
#define ELEVATOR_ACCELERATION 8000.0
#define ELEVATOR_P 0.00003
#define ELEVATOR_I 0.0
#define ELEVATOR_MAX_ACCUM 0.0
#define ELEVATOR_I_ZONE 0.0 // Encoder ticks
#define ELEVATOR_D 0.0
#define ELEVATOR_F 0.00018
#define ELEVATOR_FF 0.3
// Climb
#define ELEVATOR_CLIMB_VELOCITY 500.0
#define ELEVATOR_CLIMB_ACCELERATION 100.0
#define ELEVATOR_CLIMB_P 0.0
#define ELEVATOR_CLIMB_I 0.0
#define ELEVATOR_CLIMB_MAX_ACCUM 0.0
#define ELEVATOR_CLIMB_I_ZONE 0.0
#define ELEVATOR_CLIMB_D 0.0
#define ELEVATOR_CLIMB_F 0.0
#define ELEVATOR_CLIMB_FF 0.0
#define ELEVATOR_CLIMB_HEIGHT 10.0 // Encoder ticks
// Safety
#define ELEVATOR_MAX_CLOSED_LOOP_HEIGHT (ELEVATOR_MAX - 0)
#define ELEVATOR_MIN_CLOSED_LOOP_HEIGHT 2.0 // Encoder ticks
#define ELEVATOR_MIN_RAW_HEIGHT 0 // Encoder ticks

/* Energy Management */
#define ELEVATOR_CONTINUOUS_CURRENT_LIMIT 40 // Amperes
#define ELEVATOR_PEAK_CURRENT_LIMIT 300 // Amperes
#define ELEVATOR_PEAK_CURRENT_DURATION 200 // Milliseconds

#define ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR 0 // Encoder ticks - Zero is always try to get to value and do not stop
#define ELEVATOR_WITHIN_SET_POINT_AMOUNT 2.5

#define ELEVATOR_OPEN_LOOP_RAMP 0.2 // Seconds
#define ELEVATOR_CLOSED_LOOP_RAMP 0.1 // Seconds

#define ELEVATOR_SAFE_DOWN 0.0 // Percent output
#define ELEVATOR_SAFE_DOWN_THRESHOLD_HEIGHT 2.0 // Encoder Ticks

#define ELEVATOR_NORMAL_PID_SLOT 0
#define ELEVATOR_CLIMB_PID_SLOT 1

#define ELEVATOR_LAND_TIME 3.0 // Seconds left in match when elevator tries to land to avoid damage

namespace garage {
    class Elevator;

    using ElevatorController=lib::SubsystemController<Elevator>;

    class RawElevatorController : public ElevatorController {
    protected:
        double m_Output = 0.0;

    public:
        RawElevatorController(std::weak_ptr<Elevator>& elevator)
                : ElevatorController(elevator, "Raw Controller") {}

        void Reset() override {
            m_Output = 0.0;
        }

        void ProcessCommand(Command& command) override;

        void Control() override;

        void SetRawOutput(double output) {
            m_Output = output;
        }
    };

    class SetPointElevatorController : public ElevatorController {
    protected:
        double m_WantedSetPoint = 0;

    public:
        SetPointElevatorController(std::weak_ptr<Elevator>& elevator)
                : ElevatorController(elevator, "Set Point Controller") {}

        void ProcessCommand(Command& command) override;

        void Control() override;

        void Reset() override {
            m_WantedSetPoint = 0.0;
        }

        void SetWantedSetPoint(double wantedSetPoint) {
            m_WantedSetPoint = wantedSetPoint;
        }
    };

    class VelocityElevatorController : public ElevatorController {
    protected:
        double m_WantedVelocity = 0.0;

    public:
        VelocityElevatorController(std::weak_ptr<Elevator>& elevator)
                : ElevatorController(elevator, "Velocity Controller") {}

        void ProcessCommand(Command& command) override;

        void Control() override;

        void Reset() override {
            m_WantedVelocity = 0.0;
        }
    };

    class SoftLandElevatorController : public ElevatorController {
    public:
        SoftLandElevatorController(std::weak_ptr<Elevator>& elevator)
                : ElevatorController(elevator, "Soft Land Controller") {}

        void Control() override;
    };

    class ClimbElevatorController : public ElevatorController {
    protected:
        void Control() override;

        void Reset() override {

        }

    public:
        ClimbElevatorController(std::weak_ptr<Elevator>& elevator)
                : ElevatorController(elevator, "Climb Elevator Controller") {}
    };

    class Elevator : public lib::ControllableSubsystem<Elevator> {
        friend class RawElevatorController;

        friend class SetPointElevatorController;

        friend class VelocityElevatorController;

        friend class SoftLandElevatorController;

        friend class ClimbElevatorController;

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
        std::shared_ptr<ClimbElevatorController> m_ClimbController;

        void ConfigSpeedControllers();

        void SetupNetworkTableEntries();

        bool ShouldUnlock(Command& command) override;

        void Update() override;

        void SpacedUpdate(Command& command) override;

    public:
        Elevator(std::shared_ptr<Robot>& robot);

        void OnPostInitialize() override;

        bool WithinPosition(double targetPosition);

        void SetWantedSetPoint(double wantedSetPoint);

        void SetRawOutput(double output);

        void SoftLand();

        void Climb();

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

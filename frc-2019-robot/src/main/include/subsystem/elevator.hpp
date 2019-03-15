#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem_controller.hpp>
#include <lib/controllable_subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#define ELEVATOR_MAX 350000 // Encoder ticks
#define ELEVATOR_MIN 0 // Encoder ticks

/* Gains and Motion Magic */
#define ELEVATOR_VELOCITY 25000 // Encoder ticks per 100 ms
#define ELEVATOR_ACCELERATION 20000 // Encoder ticks per 100 ms per 100 ms
#define ELEVATOR_P 0.0
#define ELEVATOR_I 0.0
#define ELEVATOR_MAX_ACCUM 0.0
#define ELEVATOR_I_ZONE 0 // Encoder ticks
#define ELEVATOR_D 0.0
#define ELEVATOR_S_CURVE_STRENGTH 3 // Value between 1-8 which determines how curved the trapezoidal motion profile is
//#define ELEVATOR_D ELEVATOR_P * 3.3
#define ELEVATOR_F 0.0 // Multiplied by velocity calculated by motion magic and added to output, does most of work
#define ELEVATOR_FF 0.0 // Percent output - Output required to hold elevator at a position, always added to motor output in closed loop
#define ELEVATOR_MAX_CLOSED_LOOP_HEIGHT (ELEVATOR_MAX - 10000)
#define ELEVATOR_MIN_CLOSED_LOOP_HEIGHT 5000 // Encoder ticks
#define ELEVATOR_MIN_RAW_HEIGHT 10000 // Encoder ticks

/* Energy Management */
#define ELEVATOR_CONTINUOUS_CURRENT_LIMIT 40 // Amperes
#define ELEVATOR_PEAK_CURRENT_LIMIT 300 // Amperes
#define ELEVATOR_PEAK_CURRENT_DURATION 200 // Milliseconds

#define ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR 0 // Encoder ticks - Zero is always try to get to value and do not stop
#define ELEVATOR_WITHIN_SET_POINT_AMOUNT 2000 // Encoder ticks

#define ELEVATOR_OPEN_LOOP_RAMP 0.4 // Seconds
#define ELEVATOR_CLOSED_LOOP_RAMP 0.1 // Seconds

#define ELEVATOR_SAFE_DOWN 0.22 // Percent output

#define ELEVATOR_MOTION_MAGIC_PID_SLOT 0

#define ELEVATOR_LAND_TIME 3.0 // Seconds left in match when elevator tries to land to avoid damage

namespace garage {
    class Elevator;

    class RawElevatorController : public lib::SubsystemController<Elevator> {
    protected:
        double m_Input = 0.0, m_Output = 0.0;

    public:
        RawElevatorController(std::weak_ptr<Elevator>& subsystem) : lib::SubsystemController<Elevator>(subsystem, "Raw Controller") {}

        void ProcessCommand(Command& command) override;

        void Control() override;

        void Reset() override;

        void SetRawOutput(double output) {
            m_Output = output;
        }
    };

    class SetPointElevatorController : public lib::SubsystemController<Elevator> {
    protected:
        int m_WantedSetPoint = 0;

    public:
        SetPointElevatorController(std::weak_ptr<Elevator>& subsystem) : lib::SubsystemController<Elevator>(subsystem, "Set Point Controller") {}

        void ProcessCommand(Command& command) override;

        void Control() override;

        void Reset() override;

        void SetWantedSetPoint(int wantedSetPoint) {
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
        int m_EncoderPosition = 0, m_EncoderVelocity = 0;
        double m_FeedForward = ELEVATOR_FF, m_MaxVelocity = ELEVATOR_VELOCITY;
        ctre::phoenix::motorcontrol::StickyFaults m_StickyFaults;
        ctre::phoenix::motorcontrol::can::TalonSRX m_ElevatorMaster{ELEVATOR_MASTER};
        ctre::phoenix::motorcontrol::can::VictorSPX
                m_ElevatorSlaveOne{ELEVATOR_SLAVE_ONE}, m_ElevatorSlaveTwo{ELEVATOR_SLAVE_TWO}, m_ElevatorSlaveThree{ELEVATOR_SLAVE_THREE};
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

        bool WithinPosition(int targetPosition);

        void SetWantedSetPoint(int wantedSetPoint);

        void SetRawOutput(double output);

        void SoftLand();

        int GetPosition() {
            return m_EncoderPosition;
        }

        int GetVelocity() {
            return m_EncoderVelocity;
        }
    };
}

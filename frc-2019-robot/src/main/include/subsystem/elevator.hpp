#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>
#include <lib/subsystem_controller.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#include <wpi/optional.h>

#define ELEVATOR_MIN 0
#define ELEVATOR_MAX 250000

/* Gains and Motion Magic */
#define ELEVATOR_VELOCITY 12000 // Units in encoder ticks per 100 ms
#define ELEVATOR_ACCELERATION 10000 // Units in encoder ticks per 100 ms per 100 ms
#define ELEVATOR_P 0.0
#define ELEVATOR_I 0.0
#define ELEVATOR_MAX_I 0.0
#define ELEVATOR_I_ZONE 0
#define ELEVATOR_D 0.0
//#define ELEVATOR_D ELEVATOR_P * 3.3
#define ELEVATOR_F 0.0
#define ELEVATOR_FF 0.0875

// Energy
#define ELEVATOR_VOLTAGE_SATURATION 12.0 // Volts
#define ELEVATOR_CONTINOUS_CURRENT_LIMIT 40 // Amperes
#define ELEVATOR_PEAK_CURRENT_LIMIT 300 // Amperes
#define ELEVATOR_PEAK_CURRENT_DURATION 200

#define ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR 0
#define ELEVATOR_WITHIN_SET_POINT_AMOUNT 1000

#define ELEVATOR_MIN_CLOSED_LOOP_HEIGHT 2500.0

#define ELEVATOR_OPEN_LOOP_RAMP 0.4
#define ELEVATOR_CLOSED_LOOP_RAMP 0.1

#define SAFE_ELEVATOR_DOWN_STRONG 0.02
#define SAFE_ELEVATOR_DOWN_WEAK 0.01
#define SOFT_LAND_ELEVATOR_POSITION_STRONG 11000
#define SOFT_LAND_ELEVATOR_POSITION_WEAK 30000

#define ELEVATOR_UP_OUTPUT 0.4
#define ELEVATOR_DOWN_OUTPUT 0.085

#define SET_POINT_SLOT_INDEX 0

namespace garage {
    class Elevator;

    using ElevatorController = lib::SubsystemController<Elevator>;

    class RawElevatorController : public ElevatorController {
    protected:
        double m_Input = 0.0, m_Output = 0.0;

    public:
        RawElevatorController(std::shared_ptr<Elevator>& subsystem) : ElevatorController(subsystem, "Raw Controller") {};

        void ProcessCommand(Command& command) override;

        void Control() override;

        void SetRawOutput(double output) {
            m_Output = output;
        }
    };

    class SetPointElevatorController : public ElevatorController {
    protected:
        int m_WantedSetPoint = 0;

    public:
        SetPointElevatorController(std::shared_ptr<Elevator>& subsystem) : ElevatorController(subsystem, "Set Point Controller") {};

        void ProcessCommand(Command& command) override;

        void Control() override;

        void Reset() override;

        void SetWantedSetPoint(int wantedSetPoint) {
            m_WantedSetPoint = wantedSetPoint;
        }
    };

    class HybridElevatorController : public ElevatorController {
    protected:
        double m_Input = 0.0, m_WantedVelocity = 0.0;

    public:
        HybridElevatorController(std::shared_ptr<Elevator>& subsystem) : ElevatorController(subsystem, "Hybrid Controller") {};

        void ProcessCommand(Command& command) override;

        void Control() override;

        void Reset() override;
    };

    class SoftLandElevatorController : public ElevatorController {
    public:
        SoftLandElevatorController(std::shared_ptr<Elevator>& subsystem) : ElevatorController(subsystem, "Soft Land Controller") {};

        void Control() override;
    };

    class Elevator : public lib::Subsystem {
        friend class RawElevatorController;

        friend class SetPointElevatorController;

        friend class HybridElevatorController;

        friend class SoftLandElevatorController;

    protected:
        int m_EncoderPosition, m_EncoderVelocity;
        double m_FeedForward = ELEVATOR_FF;
        ctre::phoenix::motorcontrol::StickyFaults m_StickyFaults;
        ctre::phoenix::motorcontrol::can::TalonSRX m_ElevatorMaster{ELEVATOR_MASTER};
        ctre::phoenix::motorcontrol::can::VictorSPX m_ElevatorSlaveOne{ELEVATOR_SLAVE_ONE}, m_ElevatorSlaveTwo{
                ELEVATOR_SLAVE_TWO}, m_ElevatorSlaveThree{ELEVATOR_SLAVE_THREE};
        std::shared_ptr<ElevatorController> m_Controller;
        std::shared_ptr<RawElevatorController> m_RawController;
        std::shared_ptr<SetPointElevatorController> m_SetPointController;
        std::shared_ptr<HybridElevatorController> m_HybridController;
        std::shared_ptr<SoftLandElevatorController> m_SoftLandController;

        bool ShouldUnlock(Command& command) override;

        void UpdateUnlocked(Command& command) override;

        void Update() override;

        void SpacedUpdate(Command& command) override;

        void OnUnlock() override;

        bool SetController(std::shared_ptr<ElevatorController> controller);

    public:
        Elevator(std::shared_ptr<Robot>& robot);

        void TeleopInit() override;

        bool WithinPosition(int targetPosition);

        void SetWantedSetPoint(int wantedSetPoint);

        void SetRawOutput(double output);

        int GetPosition() {
            return m_EncoderPosition;
        }

        int GetVelocity() {
            return m_EncoderVelocity;
        }
    };
}

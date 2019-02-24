#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>
#include <lib/subsystem_controller.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#define ELEVATOR_MIN 0
#define ELEVATOR_MAX 300000
#define ELEVATOR_VELOCITY 10000
#define ELEVATOR_ACCELERATION 5000
#define ELEVATOR_P 0.005
#define ELEVATOR_I 0
#define ELEVATOR_I_ZONE 1000
#define ELEVATOR_D ELEVATOR_P * 13.0
#define ELEVATOR_F 1023.0 / ELEVATOR_VELOCITY
#define ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR 2000

#define ELEVATOR_MIN_CLOSED_LOOP_HEIGHT 5000.0

#define ELEVATOR_OPEN_LOOP_RAMP 0.4
#define ELEVATOR_CLOSED_LOOP_RAMP 0.0

#define SAFE_ELEVATOR_DOWN_STRONG 0.11
#define SAFE_ELEVATOR_DOWN_WEAK 0.09

#define SOFT_LAND_ELEVATOR_POSITION_STRONG 11000
#define SOFT_LAND_ELEVATOR_POSITION_WEAK 30000

#define ELEVATOR_UP_OUTPUT 0.6
#define ELEVATOR_DOWN_OUTPUT 0.08

#define SET_POINT_SLOT_INDEX 0

namespace garage {
    class Elevator;

    using ElevatorController = lib::SubsystemController<Elevator>;

    class RawElevatorController : public ElevatorController {
    public:
        using SubsystemController::SubsystemController;

        void Control(Command& command) override;
    };

    class SetPointElevatorController : public ElevatorController {
    protected:
        int m_WantedSetPoint = 0, m_LastSetPointSet = 0;

    public:
        using SubsystemController::SubsystemController;

        void Control(Command& command) override;

        void SetWantedSetPoint(int wantedSetPoint) {
            m_WantedSetPoint = wantedSetPoint;
        }
    };

    class HybridElevatorController : public ElevatorController {
    public:
        using SubsystemController::SubsystemController;

        void Control(Command& command) override;
    };

    class SoftLandElevatorController : public ElevatorController {
    public:
        using SubsystemController::SubsystemController;

        void Control(Command& command) override;
    };

    class Elevator : public lib::Subsystem {
    private:
        bool m_IsLimitSwitchDown = true;
        int m_EncoderPosition, m_EncoderVelocity;
        ctre::phoenix::motorcontrol::StickyFaults m_StickyFaults;
        ctre::phoenix::motorcontrol::can::TalonSRX m_ElevatorMaster{ELEVATOR_MASTER};
        ctre::phoenix::motorcontrol::can::VictorSPX m_ElevatorSlaveOne{ELEVATOR_SLAVE_ONE}, m_ElevatorSlaveTwo{
                ELEVATOR_SLAVE_TWO}, m_ElevatorSlaveThree{ELEVATOR_SLAVE_THREE};
        std::shared_ptr<ElevatorController> m_Controller;
        std::shared_ptr<RawElevatorController> m_RawController;
        std::shared_ptr<SetPointElevatorController> m_SetPointController;
        std::shared_ptr<HybridElevatorController> m_HybridController;
        std::shared_ptr<SoftLandElevatorController> m_SoftLandController;

        friend class RawElevatorController;

        friend class SetPointElevatorController;

        friend class HybridElevatorController;

        friend class SoftLandElevatorController;

    protected:
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

        int GetElevatorPosition();

        void SetElevatorWantedSetPoint(int wantedSetPoint);
    };
}

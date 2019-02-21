#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#define ELEVATOR_MIN 1000
#define ELEVATOR_MAX 300000
#define ELEVATOR_VELOCITY 10000
#define ELEVATOR_ACCELERATION 5000
#define ELEVATOR_P 0.01
#define ELEVATOR_I 0.0
#define ELEVATOR_D ELEVATOR_P * 13.0
#define ELEVATOR_F 1023.0 / ELEVATOR_VELOCITY
#define ELEVATOR_MIN_CLOSED_LOOP_HEIGHT 5000.0

#define ELEVATOR_OPEN_LOOP_RAMP 0.4

#define SAFE_ELEVATOR_DOWN_STRONG 0.11
#define SAFE_ELEVATOR_DOWN_WEAK 0.09

#define KILL_ELEVATOR_POSITION_STRONG 11000
#define KILL_ELEVATOR_POSITION_WEAK 30000

#define ELEVATOR_UP_OUTPUT 0.6
#define ELEVATOR_DOWN_OUTPUT 0.08

#define SET_POINT_SLOT_INDEX 0

namespace garage {
    enum class ElevatorControlMode {
        k_Manual, k_SetPoint, k_Hybrid, k_Idle, k_SoftLand
    };

    class Elevator : public lib::Subsystem {
    private:
        ElevatorControlMode m_DefaultControlMode = ElevatorControlMode::k_Manual, m_ControlMode = m_DefaultControlMode;
        bool m_IsLimitSwitchDown = true, m_FirstLimitSwitchHit = true;
        double m_Output = 0.0;
        int m_EncoderPosition, m_WantedSetPoint = ELEVATOR_MIN, m_LastSetPoint = m_WantedSetPoint;
        ctre::phoenix::motorcontrol::can::TalonSRX m_ElevatorMaster{ELEVATOR_MASTER};
        ctre::phoenix::motorcontrol::can::VictorSPX m_ElevatorSlaveOne{ELEVATOR_SLAVE_ONE}, m_ElevatorSlaveTwo{
                ELEVATOR_SLAVE_TWO}, m_ElevatorSlaveThree{ELEVATOR_SLAVE_THREE};

    protected:
        void ProcessCommand(Command& command) override;

        void Update() override;

        void SpacedUpdate(Command& command) override;

    public:
        Elevator(std::shared_ptr<Robot>& robot);

        void TeleopInit() override;

        int GetElevatorPosition();

        void SetElevatorWantedPosition(int wantedPosition);
    };
}

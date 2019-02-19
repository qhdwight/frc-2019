#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#define ELEVATOR_MIN 1000
#define ELEVATOR_MAX 300000
#define ELEVATOR_VELOCITY 10000
#define ELEVATOR_ACCELERATION 5000
#define ELEVATOR_P 0.00001
#define ELEVATOR_I 0.0
#define ELEVATOR_D ELEVATOR_P * 13.0
#define ELEVATOR_F 1023.0 / ELEVATOR_VELOCITY
#define ELEVATOR_MIN_SETPOINT_HEIGHT 3000.0

namespace garage {
    enum ElevatorControlMode {
        MANUAL, SETPOINT, HYBRID
    };
    class Elevator : public lib::Subsystem {
    private:
        ElevatorControlMode m_ControlMode = ElevatorControlMode::MANUAL;
        bool m_KillActivated = false, m_FirstLimitSwitchHit = true;
        int m_WantedSetPoint = ELEVATOR_MIN, m_LastSetPoint = m_WantedSetPoint;
        ctre::phoenix::motorcontrol::can::TalonSRX m_ElevatorMaster{ELEVATOR_MASTER};
        ctre::phoenix::motorcontrol::can::VictorSPX m_ElevatorSlaveOne{ELEVATOR_SLAVE_ONE}, m_ElevatorSlaveTwo{
                ELEVATOR_SLAVE_TWO}, m_ElevatorSlaveThree{ELEVATOR_SLAVE_THREE};
    public:
        Elevator(std::shared_ptr<Robot>& robot);

        void ExecuteCommand(Command& command) override;

        void TeleopInit() override;

        int GetElevatorPosition();

        void SetElevatorWantedPosition(int wantedPosition);
    };
}

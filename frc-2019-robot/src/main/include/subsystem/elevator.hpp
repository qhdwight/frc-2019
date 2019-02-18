#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#define ELEVATOR_MIN 1000
#define ELEVATOR_MAX 340000
#define ELEVATOR_VELOCITY 25000
#define ELEVATOR_ACCELERATION 30000
#define ELEVATOR_P 0.05
#define ELEVATOR_I 0.0
#define ELEVATOR_D ELEVATOR_P * 10.0
#define ELEVATOR_F 1023.0 / ELEVATOR_VELOCITY

namespace garage {
    class Elevator : public lib::Subsystem {
    private:
        int m_WantedPosition = ELEVATOR_MIN;
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

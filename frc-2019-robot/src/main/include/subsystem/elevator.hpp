#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

namespace garage {
    class Elevator : public lib::Subsystem {
    private:
        ctre::phoenix::motorcontrol::can::TalonSRX m_ElevatorMaster{ELEVATOR_MASTER};
        ctre::phoenix::motorcontrol::can::VictorSPX m_ElevatorSlaveOne{ELEVATOR_SLAVE_ONE}, m_ElevatorSlaveTwo{
                ELEVATOR_SLAVE_TWO}, m_ElevatorSlaveThree{ELEVATOR_SLAVE_THREE};
    public:
        Elevator(std::shared_ptr<Robot>& robot);

        void ExecuteCommand(Command& command) override;
    };
}

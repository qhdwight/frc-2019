#include <subsystem/elevator.hpp>

#include <robot.hpp>

namespace garage {
    Elevator::Elevator(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
        m_ElevatorMaster.ConfigFactoryDefault();
        m_ElevatorMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
//        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, 0.0);
        m_ElevatorMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);
        m_ElevatorSlaveOne.Follow(m_ElevatorMaster);
        m_ElevatorSlaveTwo.Follow(m_ElevatorMaster);
        m_ElevatorSlaveThree.Follow(m_ElevatorMaster);
        m_ElevatorSlaveTwo.SetInverted(ctre::phoenix::motorcontrol::InvertType::OpposeMaster);
        m_ElevatorSlaveThree.SetInverted(ctre::phoenix::motorcontrol::InvertType::OpposeMaster);
//        m_ElevatorSRX.ConfigMotionAcceleration();
//        m_ElevatorSRX.ConfigMotionCruiseVelocity();
//        m_ElevatorSRX.Config_kF();
//        m_ElevatorSRX.Config_kP();
//        m_ElevatorSRX.Config_kI();
//        m_ElevatorSRX.Config_kD();
        m_Robot->GetNetworkTable()->PutNumber("Elevator Strength", 0.0);
    }

    void Elevator::ExecuteCommand(Command& command) {
        const double strength = m_Robot->GetNetworkTable()->GetNumber("Elevator Strength", 0.0), output = command.test * strength;
        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output);
        m_Robot->GetNetworkTable()->PutNumber("Elevator Encoder", m_ElevatorMaster.GetSelectedSensorPosition(0));
        m_Robot->GetNetworkTable()->PutNumber("Elevator Output", output);
        m_Robot->GetNetworkTable()->PutNumber("Elevator Amperage", m_ElevatorMaster.GetOutputCurrent());
        Subsystem::ExecuteCommand(command);
    }
}
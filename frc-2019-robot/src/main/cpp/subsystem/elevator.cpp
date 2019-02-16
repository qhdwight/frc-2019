#include <subsystem/elevator.hpp>

#include <robot.hpp>

namespace garage {
    Elevator::Elevator(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
        m_ElevatorMaster.ConfigFactoryDefault();
        m_ElevatorMaster.SetInverted(ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput);
        m_ElevatorMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveOne.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveTwo.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveThree.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
//        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, 0.0);
        m_ElevatorMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);
        m_ElevatorMaster.ConfigClosedLoopPeakOutput(0, 0.3);
        m_ElevatorMaster.ConfigOpenloopRamp(0.2);
        m_ElevatorSlaveOne.Follow(m_ElevatorMaster);
        m_ElevatorSlaveTwo.Follow(m_ElevatorMaster);
        m_ElevatorSlaveThree.Follow(m_ElevatorMaster);
        m_ElevatorSlaveOne.SetInverted(ctre::phoenix::motorcontrol::InvertType::OpposeMaster);
        m_ElevatorSlaveTwo.SetInverted(ctre::phoenix::motorcontrol::InvertType::OpposeMaster);
        m_ElevatorSlaveThree.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        m_ElevatorMaster.ConfigMotionAcceleration(1000);
        m_ElevatorMaster.ConfigMotionCruiseVelocity(4000);
        m_ElevatorMaster.Config_kF(0, 1023.0 / 4000.0);
        m_ElevatorMaster.Config_kP(0, 0.2);
        m_ElevatorMaster.Config_kI(0, 0.0);
        m_ElevatorMaster.Config_kD(0, 0.0);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Elevator Strength", 0.2);
    }

    void Elevator::TeleopInit() {
        m_ElevatorMaster.SetSelectedSensorPosition(0);
    }

    void Elevator::ExecuteCommand(Command& command) {
        command.elevatorPosition = 120000;
        if (command.button)
            m_ElevatorMaster.SetSelectedSensorPosition(0);
//        const double strength = m_Robot->GetNetworkTable()->GetNumber("Elevator Strength", 0.2), output = command.test > 0.25 ? strength : 0.0;
//        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output);
        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, command.elevatorPosition);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Elevator Encoder", m_ElevatorMaster.GetSelectedSensorPosition(0));
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Elevator Wanted Position", command.elevatorPosition);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Elevator Output", m_ElevatorMaster.GetMotorOutputPercent());
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Elevator Master Amperage", m_ElevatorMaster.GetOutputCurrent());
        Subsystem::ExecuteCommand(command);
    }
}
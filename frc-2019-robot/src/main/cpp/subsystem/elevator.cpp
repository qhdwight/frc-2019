#include <subsystem/elevator.hpp>

#include <robot.hpp>

namespace garage {
    Elevator::Elevator(std::shared_ptr<Robot>& robot) : Subsystem(robot) {
        m_ElevatorMaster.ConfigFactoryDefault();
        m_ElevatorSlaveOne.ConfigFactoryDefault();
        m_ElevatorSlaveTwo.ConfigFactoryDefault();
        m_ElevatorSlaveThree.ConfigFactoryDefault();
        m_ElevatorMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);
        // Set brake mode
        m_ElevatorMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveOne.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveTwo.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveThree.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
//        m_ElevatorMaster.ConfigPeakOutputForward(0.75);
//        m_ElevatorMaster.ConfigPeakOutputReverse(0.5);
        m_ElevatorMaster.ConfigClosedLoopPeakOutput(0, 0.75);
        m_ElevatorMaster.ConfigOpenloopRamp(0.2);
        // Configure following and inversion
        m_ElevatorSlaveOne.Follow(m_ElevatorMaster);
        m_ElevatorSlaveTwo.Follow(m_ElevatorMaster);
        m_ElevatorSlaveThree.Follow(m_ElevatorMaster);
        m_ElevatorMaster.SetInverted(ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput);
        m_ElevatorSlaveOne.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        m_ElevatorSlaveTwo.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        m_ElevatorSlaveThree.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Acceleration", ELEVATOR_ACCELERATION);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Velocity", ELEVATOR_VELOCITY);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/kF", ELEVATOR_F);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/kD", ELEVATOR_D);
    }

    void Elevator::TeleopInit() {
        m_ElevatorMaster.SetSelectedSensorPosition(0);
    }

    void Elevator::ExecuteCommand(Command& command) {
        m_ElevatorMaster.ConfigMotionAcceleration(static_cast<int>(m_Robot->GetNetworkTable()->GetNumber("Elevator/Acceleration", ELEVATOR_ACCELERATION)));
        m_ElevatorMaster.ConfigMotionCruiseVelocity(static_cast<int>(m_Robot->GetNetworkTable()->GetNumber("Elevator/Velocity", ELEVATOR_VELOCITY)));
        m_ElevatorMaster.Config_kF(0, m_Robot->GetNetworkTable()->GetNumber("Elevator/kF", ELEVATOR_F));
        m_ElevatorMaster.Config_kD(0, m_Robot->GetNetworkTable()->GetNumber("Elevator/kD", ELEVATOR_D));
        if (command.button)
            m_ElevatorMaster.SetSelectedSensorPosition(0);
        const int encoderPosition = m_ElevatorMaster.GetSelectedSensorPosition(0);
        if (encoderPosition < ELEVATOR_MAX && encoderPosition > 1000) {
            m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, command.elevatorPosition);
        } else {
            m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        }
//        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, command.test);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Elevator Encoder", m_ElevatorMaster.GetSelectedSensorPosition(0));
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Elevator Wanted Position", command.elevatorPosition);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Elevator Output", m_ElevatorMaster.GetMotorOutputPercent());
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Elevator Master Amperage", m_ElevatorMaster.GetOutputCurrent());
        Subsystem::ExecuteCommand(command);
    }
}
#include <subsystem/elevator.hpp>

namespace garage {
    void Elevator::Initialize() {
        m_ElevatorMaster.ConfigFactoryDefault();
        m_ElevatorMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, 0.1);
        m_ElevatorMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);
//        m_ElevatorSRX.ConfigMotionAcceleration();
//        m_ElevatorSRX.ConfigMotionCruiseVelocity();
//        m_ElevatorSRX.Config_kF();
//        m_ElevatorSRX.Config_kP();
//        m_ElevatorSRX.Config_kI();
//        m_ElevatorSRX.Config_kD();
    }

    void Elevator::ExecuteCommand(Command& command) {
        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, command.flipper);
    }
}
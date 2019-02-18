#include <subsystem/elevator.hpp>

#include <robot.hpp>

namespace garage {
    Elevator::Elevator(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Elevator") {
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
        m_ElevatorMaster.ConfigOpenloopRamp(0.2);
        // Configure following and inversion
        m_ElevatorSlaveOne.Follow(m_ElevatorMaster);
        m_ElevatorSlaveTwo.Follow(m_ElevatorMaster);
        m_ElevatorSlaveThree.Follow(m_ElevatorMaster);
        m_ElevatorMaster.SetInverted(ctre::phoenix::motorcontrol::InvertType::None);
        m_ElevatorSlaveOne.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        m_ElevatorSlaveTwo.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        m_ElevatorSlaveThree.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Acceleration", ELEVATOR_ACCELERATION);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Velocity", ELEVATOR_VELOCITY);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/kP", ELEVATOR_P);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/kD", ELEVATOR_D);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/kF", ELEVATOR_F);
        m_ElevatorMaster.ConfigMotionAcceleration(ELEVATOR_ACCELERATION);
        m_ElevatorMaster.ConfigMotionCruiseVelocity(ELEVATOR_VELOCITY);
        m_ElevatorMaster.Config_kF(0, ELEVATOR_F);
        m_ElevatorMaster.Config_kP(0, ELEVATOR_P);
        m_ElevatorMaster.Config_kD(0, ELEVATOR_D);
//        m_Robot->GetNetworkTable()->GetEntry("Elevator/Acceleration").AddListener([&](const nt::EntryNotification& notification) {
//            m_ElevatorMaster.ConfigMotionAcceleration(static_cast<int>(notification.value->GetDouble()));
//        }, 0xFF);
//        m_Robot->GetNetworkTable()->GetEntry("Elevator/Velocity").AddListener([&](const nt::EntryNotification& notification) {
//            m_ElevatorMaster.ConfigMotionCruiseVelocity(static_cast<int>(notification.value->GetDouble()));
//        }, 0xFF);
//        m_Robot->GetNetworkTable()->GetEntry("Elevator/kF").AddListener([&](const nt::EntryNotification& notification) {
//            m_ElevatorMaster.Config_kF(0, static_cast<int>(notification.value->GetDouble()));
//        }, 0xFF);
//        m_Robot->GetNetworkTable()->GetEntry("Elevator/kD").AddListener([&](const nt::EntryNotification& notification) {
//            m_ElevatorMaster.Config_kD(0, static_cast<int>(notification.value->GetDouble()));
//        }, 0xFF);
//        m_Robot->GetNetworkTable()->GetEntry("Elevator/kP").AddListener([&](const nt::EntryNotification& notification) {
//            m_ElevatorMaster.Config_kP(0, static_cast<int>(notification.value->GetDouble()));
//        }, 0xFF);
    }

    void Elevator::TeleopInit() {
        m_ElevatorMaster.SetSelectedSensorPosition(0);
        m_WantedPosition = 1000;
    }

    void Elevator::ExecuteCommand(Command& command) {
        const bool isLimitSwitch = static_cast<const bool>(m_ElevatorMaster.GetSensorCollection().IsRevLimitSwitchClosed());
        if (isLimitSwitch)
            m_ElevatorMaster.SetSelectedSensorPosition(0);
        if (!m_IsLocked)
            m_WantedPosition = command.elevatorPosition;
        if (command.button)
            m_ElevatorMaster.SetSelectedSensorPosition(0);
        const int encoderPosition = m_ElevatorMaster.GetSelectedSensorPosition(0);
        if (encoderPosition < ELEVATOR_MAX - 1000 && m_WantedPosition > 2000) {
            m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, m_WantedPosition);
        } else {
            m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.15);
        }
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Encoder", m_ElevatorMaster.GetSelectedSensorPosition(0));
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Wanted Position", command.elevatorPosition);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Output", m_ElevatorMaster.GetMotorOutputPercent());
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Master Amperage", m_ElevatorMaster.GetOutputCurrent());
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Limit Switch", isLimitSwitch ? 1.0 : 0.0);
        Subsystem::ExecuteCommand(command);
    }

    int Elevator::GetElevatorPosition() {
        return m_ElevatorMaster.GetSelectedSensorPosition(0);
    }

    void Elevator::SetElevatorWantedPosition(int wantedPosition) {
        m_WantedPosition = wantedPosition;
    }
}